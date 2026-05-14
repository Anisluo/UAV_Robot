#!/usr/bin/env python3
"""proc_piper — UAV_Robot gen-2 arm process.

Replaces proc_arm + proc_gripper for the AgileX Piper 6-DOF arm.

  Listens on : /tmp/uav_proc_piper.sock  (JSON-RPC 2.0, newline delimited)
  Drives    : Piper via piper_sdk over SocketCAN (default can_piper, 1Mbps)
  Threading : main = RPC server, bg = continuous 200Hz CAN heartbeat

RPC contract:
  - Mirrors proc_arm's `arm.*` methods so HostGUI / gateway don't need changes.
  - Plus `piper.*` extras for features specific to Piper (continuous gripper,
    MoveL, MoveC, etc.).

Handshake quirk (Piper V1.8-2 firmware): on connect we *must* push the
controller through STANDBY before CAN_CTRL will latch. See:
   memory/piper_can_ctrl_handshake.md

Safe shutdown (user preference): on SIGTERM we MoveJ to all-zero but do NOT
call DisableArm — motors stay energised holding the zero pose. See:
   memory/piper_safe_shutdown.md
"""
import argparse, json, os, signal, socket, sys, threading, time, traceback
from typing import Any, Dict, List, Optional, Tuple

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    sys.stderr.write("ERROR: piper_sdk not installed. pip install piper-sdk\n")
    sys.exit(1)

# ─── constants ───────────────────────────────────────────────────────────
F      = 1000.0          # SDK unit factor (0.001° / 0.001mm / 0.001 N·m / μm)
# 100Hz heartbeat = 10 ms worst-case slider→motion latency. Was 200Hz, but
# 200Hz × 5 CAN frames per cycle saturated Python's GIL and stalled the RPC
# handlers (6–9 ms median). At 100Hz, with the cache layer added below and
# GripperCtrl skipped when unchanged, we send 2–3 frames/cycle and the
# RX-parser thread still gets ~70 % of one core (its own CPU cost, not ours).
HZ     = 100
PERIOD = 1.0 / HZ        # 10 ms

# Piper hardware joint limits (deg). Used to clamp user inputs and to
# return sane "not reachable" errors before the firmware does.
JOINT_LIMITS_DEG = [
    (-150.0, +150.0),    # J1
    (   0.0, +180.0),    # J2
    (-170.0,    0.0),    # J3
    (-100.0, +100.0),    # J4
    ( -70.0,  +70.0),    # J5
    (-180.0, +180.0),    # J6
]
GRIPPER_MAX_MM = 80.0   # full open
GRIPPER_MIN_MM =  0.0
PARK_J_INT     = [0]*6   # all-zero "safe rest" pose (0.001° units)

# Move modes (MotionCtrl_2 byte 1):
MODE_P, MODE_J, MODE_L, MODE_C = 0x00, 0x01, 0x02, 0x03

# Default config (overridable via env / CLI)
DEFAULT_IFACE   = os.environ.get("UAV_PIPER_CAN_IFACE", "can_piper")
DEFAULT_SOCKET  = os.environ.get("UAV_PROC_PIPER_SOCK", "/tmp/uav_proc_piper.sock")
DEFAULT_SPEED   = int(os.environ.get("UAV_PIPER_SPEED", "30"))


def log(level: str, msg: str) -> None:
    """Single-line stdout log, journal-friendly (no buffering)."""
    print(f"[proc_piper][{level}] {msg}", flush=True)


# ─── controller wraps SDK + maintains heartbeat ─────────────────────────
class PiperController:
    """All Piper interaction goes through this class.

    A single background thread (heartbeat) pushes the *current target*
    (joints or Cartesian) at HZ. RPC handlers just mutate the target.
    """

    def __init__(self, iface: str, speed: int):
        self.iface = iface
        self.speed = speed
        self.lock = threading.Lock()

        # Mutable target state — guarded by self.lock
        self.move_mode = MODE_J         # 0x01 MOVE_J
        self.target_joints: List[int] = [0]*6   # 0.001°
        self.target_cart:  Optional[List[int]] = None   # [X,Y,Z,RX,RY,RZ] in 0.001 mm/deg, None ⇒ use joints
        self.gripper_target_um = 0      # μm (0.001 mm)
        self.gripper_effort    = 1000   # mN·m
        # Gripper "dirty" tracking: at 50Hz heartbeat, repeatedly sending the
        # same GripperCtrl 50×/s saturates GIL for no benefit (Piper holds the
        # commanded position itself). Only send when the target changes — and
        # re-send every ~1 s as a safety heartbeat in case a frame was lost.
        self._gripper_dirty       = True
        self._gripper_last_send_t = 0.0

        # Feedback cache, populated by heartbeat. RPC handlers return these
        # instead of calling SDK getters, so they don't contend for the SDK's
        # internal mutex with the RX-parser thread.
        # Cache refresh runs every other heartbeat cycle (≈50 Hz) regardless
        # of HZ — refreshing 100 ×/s was pushing the SDK's RX thread to 90 %
        # CPU because every refresh acquires the SDK mutex 3 times.
        self._cache_lock = threading.Lock()
        self._cache_joints: List[int]   = [0]*6           # 0.001°
        self._cache_pose:   List[int]   = [0]*6           # 0.001 mm/deg
        self._cache_status: Dict[str, Any] = {
            "ok": True, "ctrl_mode": 0, "arm_status": 0, "mode_feed": 0,
            "teach_status": 0, "motion_status": 0, "error_code": 0,
            "heartbeat_alive": False, "speed_pct": speed,
        }
        self._cache_skip_counter = 0
        # Auto-recovery: count consecutive cache snapshots where ctrl_mode
        # is NOT CAN_CTRL(1). When the operator triggers the physical
        # drag-teach button on the arm body and then exits, ctrl_mode
        # gets stuck oscillating STANDBY↔CAN_CTRL forever — our heartbeat
        # alone can't pull it back without EmergencyStop(0x02) first.
        self._nonctrl_streak    = 0
        self._last_recovery_t   = 0.0
        # Teach-mode latch. When True the heartbeat suppresses its CAN_CTRL
        # refresh + JointCtrl + GripperCtrl sends so the operator can drag
        # the arm by hand without us fighting them every 10 ms. Set by the
        # RPC handler (piper.set_teach_mode) and ALSO inferred from the
        # firmware's teach_status feedback — so pressing the physical drag-
        # teach button on the arm body works without any RPC call.
        self._teach_active = False
        # Emergency-stop latch. When the operator hits 急停, EmergencyStop
        # (0x01) brakes the motors and the firmware enters a halted state
        # (ctrl_mode != CAN_CTRL). The heartbeat MUST mute its CAN_CTRL
        # refresh — otherwise it would silently cancel the brake. Cleared
        # either by 急停(enable=False), or automatically after the
        # deceleration grace period via _estop_auto_recover_at.
        self._estop_engaged = False
        # Auto-recover deadline. After EmergencyStop(0x01), the arm needs
        # ~1-2 s to actually decelerate to a halt. Once it's settled it's
        # SAFE to send EmergencyStop(0x02)+handshake to restore CAN_CTRL.
        # Doing it sooner cancels the in-flight brake → arm free-falls
        # (operator observed: "缓慢降落一会，然后急速掉下去"). Set when
        # estop engages, checked by heartbeat.
        self._estop_auto_recover_at: float = 0.0

        # Lifecycle
        self.connected = False
        self.heartbeat_alive = False
        self._stop = threading.Event()
        self._hb_thread: Optional[threading.Thread] = None

        # SDK handle
        self.p: Optional[C_PiperInterface_V2] = None

    # ── connection & handshake ──────────────────────────────────────────
    def connect(self) -> None:
        """Open CAN, perform V1.8-2 handshake, start heartbeat."""
        log("INFO", f"connecting iface={self.iface} speed={self.speed}%")
        # start_sdk_joint_limit=False to bypass SDK 0.6.1 silent-drop bug
        self.p = C_PiperInterface_V2(
            self.iface,
            start_sdk_joint_limit=False,
            start_sdk_gripper_limit=False,
        )
        self.p.ConnectPort()
        time.sleep(0.4)

        # Seed target with current pose so heartbeat doesn't jerk the arm
        j = self.p.GetArmJointMsgs().joint_state
        self.target_joints = [j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6]

        # V1.8-2 firmware handshake. Sequence verified to also recover from
        # the post-teach "stuck oscillating STANDBY/CAN_CTRL" state, so it's
        # safe to reuse for both startup and runtime recovery.
        self._do_full_handshake()
        self.connected = True
        log("INFO", "handshake complete")

        # Start heartbeat
        self._stop.clear()
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True, name="piper-hb")
        self._hb_thread.start()
        log("INFO", "heartbeat thread started")

    def shutdown(self) -> None:
        """Park to zero, stop heartbeat. Motors STAY enabled."""
        log("INFO", "shutdown: parking to zero (motors stay enabled)")
        # Switch to joint mode + zero target — let heartbeat carry the arm
        with self.lock:
            self.move_mode = MODE_J
            self.target_joints = list(PARK_J_INT)
            self.target_cart = None
        # Wait up to 8s for the arm to actually reach zero
        t0 = time.time()
        while time.time() - t0 < 8.0:
            j = self._read_joints()
            if j and all(abs(v) < 300 for v in j):
                log("INFO", f"parked at zero after {time.time()-t0:.2f}s")
                break
            time.sleep(0.05)
        else:
            log("WARN", "park timeout — arm did NOT reach zero")
        # Halt heartbeat
        self._stop.set()
        if self._hb_thread:
            self._hb_thread.join(timeout=1.0)
        self.heartbeat_alive = False
        log("INFO", "shutdown complete (arm idle at zero, still energised)")

    def _do_full_handshake(self) -> None:
        """Drive Piper into a stable CAN_CTRL state.

        Used both on startup and on runtime recovery (after crash / e-stop
        / drag-teach exit). The firmware can be in any of several latched
        fault states, so we clear each one in order before re-arming:

          1. MotionCtrl_1(0x02, 0, 0)   — clear COLLISION latch (arm_status
                                          0x07). Firmware silently rejects
                                          CAN_CTRL transitions while this
                                          bit is set; without this step,
                                          'click 使能/恢复 after 撞机 →
                                          nothing happens' (user report).
          2. EmergencyStop(0x02)         — Resume from any halt/teach. Clears
                                          the "abnormal" flag the firmware
                                          sets when teach mode terminates;
                                          without this a post-teach arm
                                          keeps oscillating STANDBY↔CAN_CTRL.
          3. MotionCtrl_2(0x00, ...)     — explicit STANDBY transit
          4. MotionCtrl_2(0x01, MODE_J)  — into CAN_CTRL
          5. EnableArm(7, 0x02)          — power on all 6 joints

        NOTE: we used to also call MasterSlaveConfig(0xFC) here. That put
        the firmware into a slave-arm state which caused the arm to DROP
        on teach-release (firmware released gravity-comp expecting master
        input). Dropped that step entirely — this arm is standalone.
        """
        log("INFO", "handshake: MotionCtrl_1(resume) — clear COLLISION latch")
        try:
            self.p.MotionCtrl_1(0x02, 0x00, 0x00); time.sleep(0.3)
        except Exception as e:
            # Older SDK versions don't expose the 3-arg form. Fall through —
            # the EmergencyStop below covers the common case.
            log("WARN", f"MotionCtrl_1 collision-clear failed: {e}")

        log("INFO", "handshake: EmergencyStop(0x02) — resume from any halt/teach")
        self.p.EmergencyStop(0x02); time.sleep(0.3)
        # Handshake = explicit operator decision to return to active
        # control. Clear the e-stop / teach latches so the heartbeat can
        # resume sending JointCtrl after the handshake completes.
        # Otherwise the operator clicks 使能, firmware shows CAN_CTRL,
        # but proc_piper keeps the heartbeat muted and the arm doesn't
        # respond to any subsequent commands.
        self._estop_engaged = False
        self._teach_active  = False
        log("INFO", "handshake: STANDBY")
        self.p.MotionCtrl_2(0x00, 0x00, 0, 0x00); time.sleep(0.3)
        log("INFO", "handshake: CAN_CTRL")
        self.p.MotionCtrl_2(0x01, MODE_J, self.speed, 0x00); time.sleep(0.3)
        log("INFO", "handshake: EnableArm")
        self.p.EnableArm(7, 0x02); time.sleep(0.3)

    # ── heartbeat thread ───────────────────────────────────────────────
    def _heartbeat_loop(self) -> None:
        self.heartbeat_alive = True
        while not self._stop.is_set():
            try:
                with self.lock:
                    mode = self.move_mode
                    speed = self.speed
                    j = list(self.target_joints)
                    c = list(self.target_cart) if self.target_cart else None
                    g_um = self.gripper_target_um
                    g_eff = self.gripper_effort
                    g_dirty = self._gripper_dirty
                    self._gripper_dirty = False
                    in_teach = self._teach_active
                    in_estop = self._estop_engaged
                # During drag-teach OR an active emergency stop the heartbeat
                # MUST yield. For teach: our 10ms MotionCtrl_2(CAN_CTRL,...)
                # + JointCtrl(target_joints) would fight the firmware's TEACH
                # state. For e-stop: a MotionCtrl_2(CAN_CTRL) refresh would
                # immediately cancel the brake (firmware treats it as
                # "operator wants control back") and the arm drops. Cache
                # refresh below still runs so HostGUI sees live state.
                if not in_teach and not in_estop:
                    # Send mode + target each cycle — Piper firmware needs
                    # this to keep CAN_CTRL latched.
                    self.p.MotionCtrl_2(0x01, mode, speed, 0x00)
                    if c is not None and mode in (MODE_P, MODE_L, MODE_C):
                        self.p.EndPoseCtrl(*c)
                    else:
                        self.p.JointCtrl(*j)
                    # Gripper: only send when the target changed, or as a 1Hz
                    # safety re-send in case a frame got lost.
                    now = time.monotonic()
                    if g_dirty or (now - self._gripper_last_send_t) > 1.0:
                        self.p.GripperCtrl(int(g_um), int(g_eff), 0x01, 0)
                        self._gripper_last_send_t = now

                # Refresh feedback cache every other heartbeat cycle (≈ HZ/2)
                # to keep RX-thread CPU sane while still feeding HostGUI a
                # ≤20 ms-stale snapshot.
                self._cache_skip_counter += 1
                if self._cache_skip_counter >= 2:
                    self._cache_skip_counter = 0
                    try:
                        js = self.p.GetArmJointMsgs().joint_state
                        ep = self.p.GetArmEndPoseMsgs().end_pose
                        sa = self.p.GetArmStatus().arm_status
                        cur_ctrl = int(sa.ctrl_mode)
                        with self._cache_lock:
                            self._cache_joints = [js.joint_1, js.joint_2, js.joint_3,
                                                  js.joint_4, js.joint_5, js.joint_6]
                            self._cache_pose   = [ep.X_axis, ep.Y_axis, ep.Z_axis,
                                                  ep.RX_axis, ep.RY_axis, ep.RZ_axis]
                            self._cache_status = {
                                "ok": True,
                                "ctrl_mode":     cur_ctrl,
                                "arm_status":    int(sa.arm_status),
                                "mode_feed":     int(sa.mode_feed),
                                "teach_status":  int(sa.teach_status),
                                "motion_status": int(sa.motion_status),
                                "error_code":    int(getattr(sa, "err_code", 0)),
                                "heartbeat_alive": True,
                                "speed_pct":     speed,
                            }
                        # Auto-recovery: if ctrl_mode has been != CAN_CTRL(1)
                        # for several cache cycles (each = 2× heartbeat ≈ 20 ms),
                        # run the full EmergencyStop+STANDBY+CAN_CTRL sequence.
                        # This catches the post-teach STANDBY↔CAN_CTRL oscillation
                        # that the operator was hitting after exiting drag-teach.
                        #
                        # BUT: if teach_status is non-zero, the operator pushed
                        # the physical drag-teach button on the arm body and
                        # IS DELIBERATELY in teach mode — they are physically
                        # moving the arm to record waypoints. Forcing it back
                        # to CAN_CTRL right now would mean we fight them every
                        # ~100 ms and the arm visibly flickers between modes.
                        # Skip recovery as long as teach is engaged; resume the
                        # streak counter only after teach_status drops to 0.
                        # Firmware teach state — true when the operator
                        # pressed the physical drag-teach button on the arm
                        # body (firmware reports ctrl_mode=0x02 TEACHING).
                        # Software path (DisableArm) reports differently,
                        # so we hit this branch primarily for the physical
                        # button workflow. Track back into _teach_active so
                        # the heartbeat above stops fighting the operator.
                        # V1.8-2 teach_status state machine (verified live):
                        #   0 = idle (never engaged this session)
                        #   1 = START_RECORDING — operator is actively dragging
                        #   2 = STOP_RECORDING  — operator released the button
                        # ctrl_mode follows a separate path: it jumps to 2
                        # on first press but stays at 2 even after release
                        # until someone (us, or 使能 button) actively pulls
                        # it back to 1.
                        #
                        # So "engaged" is teach_status == 1 ONLY. The 1→2
                        # transition is our cue that the operator just
                        # finished dragging — that's when we run the soft
                        # re-engage to drop ctrl_mode back to 1.
                        # V1.8-2 teach_status state machine, all three values:
                        #   0 = idle      — heartbeat resumes normal control
                        #   1 = dragging  — heartbeat muted, operator using arm
                        #   2 = released  — heartbeat STAYS muted. Firmware
                        #                   is still in ctrl_mode=0x02 with
                        #                   gravity-comp holding the arm up
                        #                   at the dragged pose. Forcing it
                        #                   back to CAN_CTRL from here (via
                        #                   any combination of commands we
                        #                   tried) caused the motors to lose
                        #                   torque mid-transition — arm
                        #                   DROPPED. Don't do that.
                        #
                        # Operator exits TEACHING manually by clicking 使能
                        # in HostGUI (full handshake) WHILE supporting the
                        # arm — the handshake's STANDBY transit causes the
                        # same drop, but the operator's hand catches it.
                        # Once handshake completes, firmware resets
                        # teach_status to 0, which is when we resume.
                        ts = int(sa.teach_status)
                        was_teaching = self._teach_active
                        if ts == 1:                          # actively dragging
                            self._teach_active = True
                            self._nonctrl_streak = 0
                        elif ts == 2:                        # released, still in TEACHING
                            # Edge-trigger: only snapshot + log the FIRST
                            # time we see the falling edge. Without this,
                            # the cache-refresh fires every 20 ms and the
                            # journal floods with the same "snapshotted"
                            # line until the operator does something else.
                            edge = was_teaching
                            # SAFETY: do NOT auto-exit TEACHING here.
                            # We tried, but our detection window is too
                            # slow — by the time we see teach_status=2,
                            # the firmware has ALREADY dropped gravity-
                            # comp (operator confirmed visible drop).
                            # Even EmergencyStop(0x01) comes too late to
                            # brake what's already free-falling.
                            #
                            # Correct workflow: operator calls
                            # piper.safe_teach_exit RPC (HostGUI button)
                            # BEFORE pressing the physical button to
                            # exit. The RPC runs the braking sequence
                            # while firmware is still in TEACHING with
                            # gravity-comp engaged → safe transition.
                            #
                            # All we do on the 1→2 edge: snapshot the
                            # dragged pose so when the operator does
                            # exit (either via safe_teach_exit or via
                            # 使能 with hand support), the resumed
                            # heartbeat holds at the dragged pose.
                            if edge and len(self._cache_joints) == 6:
                                snap = list(self._cache_joints)
                                with self.lock:
                                    self.target_joints = snap
                                    self.target_cart   = None
                                    self.move_mode     = MODE_J
                                log("INFO",
                                    f"teach button-release detected (ts=2): "
                                    f"pose snapshotted {snap}. STAYING in "
                                    f"TEACHING — use piper.safe_teach_exit "
                                    f"RPC or 使能 to leave safely.")
                            self._teach_active = True        # keep heartbeat muted
                            self._nonctrl_streak = 0
                        else:                                # ts == 0, idle
                            self._teach_active = False
                            if cur_ctrl != 1:
                                self._nonctrl_streak += 1
                            else:
                                self._nonctrl_streak = 0
                        # No general auto-recovery (post-teach STANDBY is
                        # left alone; operator clicks 使能 to come back).
                        # But e-stop has a deadline-based auto-recovery:
                        # after the brake-deceleration grace period, the
                        # arm is fully stopped — safe to re-handshake and
                        # restore CAN_CTRL automatically so the operator's
                        # next move/home command works without first
                        # clicking 恢复. The grace period is the critical
                        # part — recovering DURING deceleration was what
                        # caused the "缓动→急速掉下去" bug.
                        if (self._estop_engaged and
                            time.monotonic() >= self._estop_auto_recover_at):
                            log("INFO", "estop grace period elapsed → auto-recover to CAN_CTRL")
                            try:
                                # _do_full_handshake also clears _estop_engaged
                                self._do_full_handshake()
                            except Exception as e:
                                log("ERROR", f"estop auto-recover failed: {e}")
                    except Exception:
                        pass   # don't kill the loop on a transient read miss
            except Exception as e:
                log("ERROR", f"heartbeat: {e}")
            time.sleep(PERIOD)
        self.heartbeat_alive = False

    # ── helpers ────────────────────────────────────────────────────────
    # Cache-backed readers — never call SDK getters, so RPC handlers don't
    # contend with the heartbeat / RX-parser threads for SDK locks.
    def _read_joints(self) -> Optional[List[int]]:
        with self._cache_lock:
            return list(self._cache_joints)

    def _read_pose(self) -> Optional[List[int]]:
        with self._cache_lock:
            return list(self._cache_pose)

    def _clamp_joint(self, joint_idx: int, deg: float) -> float:
        lo, hi = JOINT_LIMITS_DEG[joint_idx]
        return max(lo, min(deg, hi))

    # ── RPC method implementations ─────────────────────────────────────
    def m_arm_home(self) -> Dict[str, Any]:
        """MoveJ to all-zero (Piper's natural rest pose)."""
        with self.lock:
            self.move_mode = MODE_J
            self.target_joints = list(PARK_J_INT)
            self.target_cart = None
        return {"ok": True}

    def m_arm_home_joint(self, joint_index: int) -> Dict[str, Any]:
        """MoveJ a single joint to 0°, leave others where they are."""
        if not (1 <= joint_index <= 6):
            return {"ok": False, "error": "joint_index out of range [1..6]"}
        cur = self._read_joints() or list(self.target_joints)
        new = list(cur)
        new[joint_index - 1] = 0
        with self.lock:
            self.move_mode = MODE_J
            self.target_joints = new
            self.target_cart = None
        return {"ok": True}

    def m_arm_stop(self) -> Dict[str, Any]:
        """Stop = hold current pose (set target = current reading)."""
        cur = self._read_joints()
        if cur is None:
            return {"ok": False, "error": "cannot read joint state"}
        with self.lock:
            self.move_mode = MODE_J
            self.target_joints = list(cur)
            self.target_cart = None
        return {"ok": True}

    def m_arm_emergency_stop(self, enable: bool) -> Dict[str, Any]:
        """Emergency stop / resume.

        enable=True  → EmergencyStop(0x01) "快速急停": motors apply brake,
                       arm settles in place. Latch _estop_engaged so the
                       heartbeat stops sending MotionCtrl_2(CAN_CTRL,…)
                       (would cancel the brake) and the auto-recovery
                       doesn't fire EmergencyStop(0x02) "resume".
        enable=False → EmergencyStop(0x02) "恢复": release halt, re-run
                       handshake to latch CAN_CTRL with current pose held.
                       Snapshot live joints into target_joints first so
                       the heartbeat's resumed JointCtrl tells the arm to
                       hold where it stopped, not snap back to old target.
        """
        try:
            if enable:
                self._estop_engaged = True
                # Auto-recover after deceleration grace period. 1.5 s is
                # enough for the brake to fully halt Piper at its typical
                # speed envelope. Heartbeat watches this deadline and runs
                # the handshake when reached.
                grace_s = float(os.environ.get("UAV_PIPER_ESTOP_AUTO_RECOVER_S", "1.5"))
                self._estop_auto_recover_at = time.monotonic() + grace_s
                self.p.EmergencyStop(0x01)
                log("INFO",
                    f"estop engaged (brake applied, heartbeat muted, "
                    f"auto-recover in {grace_s:.1f}s)")
            else:
                # Snapshot pose so resumed heartbeat holds, doesn't yank
                with self._cache_lock:
                    snap = list(self._cache_joints)
                if len(snap) == 6:
                    with self.lock:
                        self.target_joints = snap
                        self.target_cart   = None
                        self.move_mode     = MODE_J
                    log("INFO", f"estop release: held at current pose {snap}")
                self.p.EmergencyStop(0x02); time.sleep(0.2)
                self.p.MotionCtrl_2(0x00, 0x00, 0, 0x00); time.sleep(0.2)
                self.p.MotionCtrl_2(0x01, MODE_J, self.speed, 0x00); time.sleep(0.2)
                self.p.EnableArm(7, 0x02)
                self._estop_engaged = False
                self._nonctrl_streak = 0
                log("INFO", "estop released, heartbeat resumes")
            return {"ok": True}
        except Exception as e:
            log("ERROR", f"emergency_stop({enable}) failed: {e}")
            return {"ok": False, "error": str(e)}

    def m_arm_set_free_mode(self, enable: bool) -> Dict[str, Any]:
        """Legacy stub. See m_arm_set_teach_mode."""
        return self.m_arm_set_teach_mode(bool(enable))

    def m_arm_set_teach_mode(self, enable: bool) -> Dict[str, Any]:
        """Software drag-teach is NOT supported on Piper V1.8-2.

        Tested every plausible CAN path on the live arm:
          - MotionCtrl_1(grag_teach_ctrl=0x01)      → flips teach_status to
            recording but keeps motor torque, arm rigid. Not useful.
          - MasterSlaveConfig(0xFA, 0, 0, 0)        → ctrl_mode stays at
            CAN_CTRL, motors locked. Not useful.
          - DisableArm(7, 0x01)                     → ctrl_mode drops to
            STANDBY, motors free — but no gravity comp, the arm sags
            (operator perceives this as "the power blinked").
          - Raw CAN frame 0x151 byte0=0x02          → firmware rejects,
            falls to STANDBY.

        The real ctrl_mode=0x02 TEACHING state (motors compliant WITH
        gravity comp) is reachable only through the physical drag-teach
        button on the arm body — that button uses an internal firmware
        path that no documented CAN command replicates.

        Workflow on this firmware: press the physical button, drag, press
        again to release. proc_piper's heartbeat detects teach_status and
        suppresses MotionCtrl_2/JointCtrl/GripperCtrl sends while the
        operator is dragging, and snapshots the live pose on release so
        the arm holds where it was left (see cache-refresh in heartbeat).
        """
        return {
            "ok": False,
            "error": "software drag-teach not supported on Piper V1.8-2; "
                     "use the physical button on the arm body",
        }

    def m_arm_set_speeds(self, joint_dps: Optional[float] = None,
                                zero_dps:  Optional[float] = None,
                                move_rpm:  Optional[int]   = None,
                                zero_rpm:  Optional[int]   = None) -> Dict[str, Any]:
        """Piper SDK uses a 1..100 'move_spd_rate_ctrl' percentage. Map the
        legacy dps / rpm conventions to that. Heuristic: 100% ≈ 200 deg/s,
        so dps→pct = dps/2. RPM unused (Piper doesn't expose RPM)."""
        new_pct = None
        if joint_dps is not None and joint_dps > 0:
            new_pct = max(1, min(100, int(round(joint_dps / 2.0))))
        elif move_rpm is not None and move_rpm > 0:
            new_pct = max(1, min(100, int(round(move_rpm / 30.0))))   # rough
        if new_pct is not None:
            with self.lock:
                self.speed = new_pct
        return {
            "ok": True,
            "joint_dps": float(self.speed * 2.0),
            "zero_dps":  float(self.speed * 2.0),
            "move_rpm":  int(self.speed * 30),
            "zero_rpm":  int(self.speed * 30),
        }

    def m_arm_get_speeds(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "joint_dps": float(self.speed * 2.0),
            "zero_dps":  float(self.speed * 2.0),
            "move_rpm":  int(self.speed * 30),
            "zero_rpm":  int(self.speed * 30),
        }

    def _check_can_ctrl(self) -> Optional[Dict[str, Any]]:
        """Return None if firmware is in CAN_CTRL and ready to accept
        JointCtrl commands. Otherwise return an error dict explaining
        what state the arm is in and how to get out.

        Used by every motion-issuing RPC (move_joint, move_joints,
        move_xyz, etc.) so they fail-fast with a clear message instead
        of silently dropping the command (heartbeat stays muted when
        the firmware is in TEACHING / STANDBY and JointCtrl never goes
        out, so the arm appears to ignore the operator).
        """
        with self._cache_lock:
            cur_ctrl = int(self._cache_status.get("ctrl_mode", 0))
        if cur_ctrl == 1 and not self._teach_active and not self._estop_engaged:
            return None
        # Diagnose for the operator
        if self._estop_engaged:
            reason = "急停已生效, 请先点 恢复 释放急停"
        elif cur_ctrl == 2 or self._teach_active:
            reason = ("机械臂在 TEACHING(物理示教)模式, 不接受运动指令。"
                      "请先扶住机械臂, 点 使能 退出示教进入 CAN_CTRL")
        elif cur_ctrl == 0:
            reason = "机械臂在 STANDBY, 请先点 使能 进入 CAN_CTRL"
        else:
            reason = f"ctrl_mode={cur_ctrl} 不是 CAN_CTRL, 请先点 使能"
        return {"ok": False, "error": reason, "ctrl_mode": cur_ctrl}

    def m_arm_move_joint(self, joint_index: int, target_deg: float) -> Dict[str, Any]:
        if not (1 <= joint_index <= 6):
            return {"ok": False, "error": "joint_index out of range"}
        err = self._check_can_ctrl()
        if err is not None:
            return err
        clamped = self._clamp_joint(joint_index - 1, target_deg)
        cur = self._read_joints() or list(self.target_joints)
        new = list(cur)
        new[joint_index - 1] = int(clamped * F)
        with self.lock:
            self.move_mode = MODE_J
            self.target_joints = new
            self.target_cart = None
        return {"ok": True}

    def m_arm_move_joints(self, angles_deg: List[float],
                          speed_ratio: float = 1.0) -> Dict[str, Any]:
        if len(angles_deg) != 6:
            return {"ok": False, "error": "need 6 joints"}
        err = self._check_can_ctrl()
        if err is not None:
            return err
        clamped = [self._clamp_joint(i, a) for i, a in enumerate(angles_deg)]
        with self.lock:
            self.move_mode = MODE_J
            self.target_joints = [int(a * F) for a in clamped]
            self.target_cart = None
        eta = self._estimate_eta_joints(clamped, speed_ratio)
        return {"ok": True, "eta_s": eta}

    def m_arm_move_xyz(self, x_mm: float, y_mm: float, z_mm: float) -> Dict[str, Any]:
        """Cartesian PTP (Piper MOVE_P). Preserves current RX/RY/RZ."""
        cur = self._read_pose()
        if cur is None:
            return {"ok": False, "error": "cannot read end pose"}
        with self.lock:
            self.move_mode = MODE_P
            self.target_cart = [int(x_mm*F), int(y_mm*F), int(z_mm*F), cur[3], cur[4], cur[5]]
        return {"ok": True, "eta_s": 3.0}

    def m_arm_move_pose6d(self, x_mm: float, y_mm: float, z_mm: float,
                          roll_deg: float, pitch_deg: float, yaw_deg: float,
                          speed_ratio: float = 1.0,
                          rotation_order: str = "zyx",
                          mode: str = "P") -> Dict[str, Any]:
        """Cartesian to (X,Y,Z,RX,RY,RZ). mode='P' (PTP, default) or 'L' (linear).
        Piper uses RX/RY/RZ Euler in the order it expects — we just pass through.
        rotation_order parameter is accepted but ignored (Piper convention)."""
        move_mode = MODE_L if mode.upper() == "L" else MODE_P
        with self.lock:
            self.move_mode = move_mode
            self.target_cart = [int(x_mm*F), int(y_mm*F), int(z_mm*F),
                                int(roll_deg*F), int(pitch_deg*F), int(yaw_deg*F)]
        return {"ok": True, "eta_s": 3.0}

    def m_arm_move_linear_xyz_rotation(self, **kw) -> Dict[str, Any]:
        kw["mode"] = "L"
        return self.m_arm_move_pose6d(**kw)

    def m_arm_get_motor_angles(self) -> List[float]:
        j = self._read_joints()
        if j is None:
            return [0.0]*6
        return [v / F for v in j]

    def m_arm_get_pose(self, rotation_order: str = "zyx") -> List[float]:
        e = self._read_pose()
        if e is None:
            return [0.0]*6
        return [e[0]/F, e[1]/F, e[2]/F, e[3]/F, e[4]/F, e[5]/F]

    def m_arm_get_transform(self) -> List[float]:
        # Piper SDK doesn't expose the 4x4 T directly; HostGUI can compute
        # from get_pose if it really needs it. Return identity for now.
        log("WARN", "arm.get_transform: returning identity (not implemented)")
        return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]

    def m_arm_set_current_zero(self, joint_index: int) -> Dict[str, Any]:
        return {"ok": False, "error": "piper has no software zero offset (mechanical only)"}

    def m_arm_reset_current_zero(self, joint_index: int) -> Dict[str, Any]:
        return {"ok": False, "error": "piper has no software zero offset (mechanical only)"}

    def m_gripper_set(self, open: bool) -> Dict[str, Any]:
        """Binary open/close. open=True → max, False → 0mm."""
        target_um = int(GRIPPER_MAX_MM * 1000) if open else int(GRIPPER_MIN_MM * 1000)
        with self.lock:
            if self.gripper_target_um != target_um:
                self._gripper_dirty = True
            self.gripper_target_um = target_um
        return {"ok": True}

    def m_arm_servo_gripper(self, angle_deg: int) -> Dict[str, Any]:
        """Legacy proc_arm servo-gripper: 0-90 deg → 0-MAX mm linearly."""
        mm = max(0.0, min(90.0, float(angle_deg))) / 90.0 * GRIPPER_MAX_MM
        target_um = int(mm * 1000)
        with self.lock:
            if self.gripper_target_um != target_um:
                self._gripper_dirty = True
            self.gripper_target_um = target_um
        return {"ok": True, "eta_s": 1.0}

    # ── piper.* extras ─────────────────────────────────────────────────
    def m_piper_set_gripper_angle(self, angle_mm: float,
                                   effort_mNm: float = 1000.0) -> Dict[str, Any]:
        """Continuous gripper: angle in mm [0, 80], effort in mN·m."""
        mm = max(GRIPPER_MIN_MM, min(GRIPPER_MAX_MM, angle_mm))
        target_um = int(mm * 1000)
        new_eff = int(effort_mNm)
        with self.lock:
            if target_um != self.gripper_target_um or new_eff != self.gripper_effort:
                self._gripper_dirty = True
            self.gripper_target_um = target_um
            self.gripper_effort = new_eff
        return {"ok": True, "angle_mm": mm}

    def m_piper_get_status(self) -> Dict[str, Any]:
        # Snapshot from cache (refreshed at 50Hz by heartbeat). Reading SDK
        # directly here was contending for the SDK mutex and adding 5 ms
        # latency per call.
        with self._cache_lock:
            out = dict(self._cache_status)
        out["heartbeat_alive"] = self.heartbeat_alive
        out["speed_pct"]       = self.speed
        return out

    def m_piper_park_zero(self) -> Dict[str, Any]:
        return self.m_arm_home()

    def m_piper_safe_teach_exit(self) -> Dict[str, Any]:
        """Safely exit drag-teach mode WITHOUT dropping the arm.

        Sequence — replicates the operator-validated manual workflow
        (急停 → 恢复 → 使能) under one RPC. Must be called while the
        firmware is STILL in TEACHING (gravity-comp engaged). Pressing
        the physical button to exit before calling this RPC is unsafe
        because the firmware drops torque the instant the button is
        released — our heartbeat cache refresh detects that ~20 ms
        later, which is too slow to brake.

        Steps:
          1. snapshot live joints into target_joints (so the resumed
             heartbeat holds where the operator dragged it)
          2. EmergencyStop(0x01) 快速急停 — controlled braking, motors
             apply enough torque to settle in place instead of free-
             falling
          3. sleep 0.3 s — let motors settle
          4. EmergencyStop(0x02) 恢复 — release halt
          5. MotionCtrl_2(0x01, MOVE_J, speed, 0) — CAN_CTRL latch
          6. EnableArm(7, 0x02) — belt-and-suspenders motor enable
          7. clear _teach_active so heartbeat takes over with snapshot
        """
        try:
            # 1. snapshot pose
            with self._cache_lock:
                snap = list(self._cache_joints)
            if len(snap) != 6:
                return {"ok": False, "error": "cache empty; cannot snapshot"}
            with self.lock:
                self.target_joints = snap
                self.target_cart   = None
                self.move_mode     = MODE_J
            log("INFO", f"safe_teach_exit: snapshot {snap}, beginning sequence")
            # 2. braked halt
            self.p.EmergencyStop(0x01)
            time.sleep(0.3)
            # 3. resume from halt
            self.p.EmergencyStop(0x02)
            time.sleep(0.1)
            # 4. CAN_CTRL
            self.p.MotionCtrl_2(0x01, MODE_J, self.speed, 0x00)
            time.sleep(0.1)
            # 5. enable
            self.p.EnableArm(7, 0x02)
            # 6. let heartbeat take over
            self._teach_active = False
            self._nonctrl_streak = 0
            log("INFO", "safe_teach_exit: completed, heartbeat resumes")
            return {"ok": True, "snap": snap}
        except Exception as e:
            log("ERROR", f"safe_teach_exit failed: {e}")
            return {"ok": False, "error": str(e)}

    def m_piper_handshake(self) -> Dict[str, Any]:
        """Force re-do the full handshake (Resume → MasterSlave → STANDBY
        → CAN_CTRL → Enable). The Resume step matters — without it, a
        post-teach arm oscillates STANDBY↔CAN_CTRL and won't latch."""
        try:
            self._do_full_handshake()
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def m_piper_move_cartesian(self, X_mm: float, Y_mm: float, Z_mm: float,
                                RX_deg: float, RY_deg: float, RZ_deg: float,
                                mode: str = "P") -> Dict[str, Any]:
        move_mode = {"P": MODE_P, "L": MODE_L}.get(mode.upper(), MODE_P)
        with self.lock:
            self.move_mode = move_mode
            self.target_cart = [int(X_mm*F), int(Y_mm*F), int(Z_mm*F),
                                int(RX_deg*F), int(RY_deg*F), int(RZ_deg*F)]
        return {"ok": True}

    # ── eta helper ─────────────────────────────────────────────────────
    def _estimate_eta_joints(self, target_deg: List[float], speed_ratio: float) -> float:
        cur = self._read_joints()
        if cur is None:
            return 3.0
        # 100% pct ≈ 200 deg/s, scale by ratio
        max_dps = max(1.0, self.speed * 2.0 * max(0.01, speed_ratio))
        deltas = [abs(target_deg[i] - cur[i]/F) for i in range(6)]
        return max(0.1, max(deltas) / max_dps + 0.3)


# ─── RPC server ──────────────────────────────────────────────────────────
class RpcServer:
    """JSON-RPC 2.0 newline-delimited Unix socket server. Threaded per client."""

    def __init__(self, sock_path: str, ctrl: PiperController):
        self.sock_path = sock_path
        self.ctrl = ctrl
        self.handlers = self._build_dispatch()
        self.srv: Optional[socket.socket] = None
        self._stop = threading.Event()

    def _build_dispatch(self) -> Dict[str, Any]:
        c = self.ctrl
        return {
            # mirror proc_arm
            "system.ping":                  lambda **kw: {"ok": True, "backend": "piper"},
            "system.get_backend":           lambda **kw: {"backend": "piper", "model": "AgileX Piper 6-DOF"},
            "arm.home":                     lambda **kw: c.m_arm_home(),
            "arm.home_joint":               lambda joint_index=None, joint=None, **_: c.m_arm_home_joint(joint_index or joint or 1),
            "arm.stop":                     lambda **kw: c.m_arm_stop(),
            "arm.emergency_stop":           lambda enable=False, **_: c.m_arm_emergency_stop(enable),
            "arm.set_free_mode":            lambda enable=False, **_: c.m_arm_set_free_mode(enable),
            "arm.set_teach_mode":           lambda enable=False, **_: c.m_arm_set_teach_mode(enable),
            "piper.set_teach_mode":         lambda enable=False, **_: c.m_arm_set_teach_mode(enable),
            "arm.set_speeds":               lambda **kw: c.m_arm_set_speeds(**kw),
            "arm.get_speeds":               lambda **kw: c.m_arm_get_speeds(),
            "arm.move_joint":               lambda joint_index=None, joint=None, target_deg=0.0, **_: c.m_arm_move_joint(joint_index or joint or 1, target_deg),
            "arm.move_joint_deg":           lambda joint_index=None, joint=None, target_deg=0.0, **_: c.m_arm_move_joint(joint_index or joint or 1, target_deg),
            "arm.move_joints":              lambda **kw: c.m_arm_move_joints(_extract_joints(kw), kw.get("speed_ratio", 1.0)),
            "arm.move_joints_deg":          lambda **kw: c.m_arm_move_joints(_extract_joints(kw), kw.get("speed_ratio", 1.0)),
            "arm.angle_mode":               lambda **kw: c.m_arm_move_joints(_extract_joints(kw), kw.get("speed_ratio", 1.0)),
            "arm.move_xyz":                 lambda x_mm=0, y_mm=0, z_mm=0, **_: c.m_arm_move_xyz(x_mm, y_mm, z_mm),
            "arm.move_pose6d":              lambda **kw: c.m_arm_move_pose6d(**_filter_kw(kw, ("x_mm","y_mm","z_mm","roll_deg","pitch_deg","yaw_deg","speed_ratio","rotation_order"))),
            "arm.move_xyz_rotation":        lambda **kw: c.m_arm_move_pose6d(**_filter_kw(kw, ("x_mm","y_mm","z_mm","roll_deg","pitch_deg","yaw_deg","speed_ratio","rotation_order"))),
            "arm.move_linear_xyz_rotation": lambda **kw: c.m_arm_move_pose6d(mode="L", **_filter_kw(kw, ("x_mm","y_mm","z_mm","roll_deg","pitch_deg","yaw_deg","rotation_order"))),
            "arm.get_motor_angles":         lambda **kw: c.m_arm_get_motor_angles(),
            # HostGUI uses the shorter "arm.get_angles" name; wraps to match
            # the get_angles reply shape proc_arm produces ({"angles":[...]})
            "arm.get_angles":               lambda **kw: {"angles": c.m_arm_get_motor_angles()},
            "arm.set_joints":               lambda **kw: c.m_arm_move_joints(_extract_joints(kw), kw.get("speed_ratio", 1.0)),
            "arm.set_current_zero":         lambda joint_index=None, joint=None, **_: c.m_arm_set_current_zero(joint_index or joint or 1),
            "arm.reset_current_zero":       lambda joint_index=None, joint=None, **_: c.m_arm_reset_current_zero(joint_index or joint or 1),
            "arm.get_pose":                 lambda rotation_order="zyx", **_: c.m_arm_get_pose(rotation_order),
            "arm.get_transform":            lambda **kw: c.m_arm_get_transform(),
            "arm.get_T":                    lambda **kw: c.m_arm_get_transform(),
            "gripper.set":                  lambda open=False, **_: c.m_gripper_set(open),
            "arm.servo_gripper":            lambda angle_deg=0, **_: c.m_arm_servo_gripper(angle_deg),

            # piper.* extras
            "piper.handshake":              lambda **kw: c.m_piper_handshake(),
            "piper.safe_teach_exit":        lambda **kw: c.m_piper_safe_teach_exit(),
            "arm.safe_teach_exit":          lambda **kw: c.m_piper_safe_teach_exit(),
            "piper.park_zero":              lambda **kw: c.m_piper_park_zero(),
            "piper.get_status":             lambda **kw: c.m_piper_get_status(),
            # Accept both the canonical "angle_mm"/"effort_mNm" names AND the
            # legacy "angle"/"force_pct" names TeachWidget v1 emitted. With
            # the legacy names, value is interpreted as mm (treat the stored
            # 0-80 number as a mm target) and force_pct 0-100 maps to
            # 0-2000 mN·m.
            "piper.set_gripper_angle":      lambda angle_mm=None, effort_mNm=None,
                                                   angle=None, force_pct=None, **_:
                c.m_piper_set_gripper_angle(
                    angle_mm if angle_mm is not None else (angle if angle is not None else 0.0),
                    effort_mNm if effort_mNm is not None else (
                        float(force_pct) * 20.0 if force_pct is not None else 1000.0)),
            "piper.move_cartesian":         lambda **kw: c.m_piper_move_cartesian(**_filter_kw(kw, ("X_mm","Y_mm","Z_mm","RX_deg","RY_deg","RZ_deg","mode"))),
        }

    def serve_forever(self) -> None:
        if os.path.exists(self.sock_path):
            os.unlink(self.sock_path)
        self.srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.srv.bind(self.sock_path)
        os.chmod(self.sock_path, 0o666)
        self.srv.listen(8)
        log("INFO", f"rpc listening on {self.sock_path}")
        while not self._stop.is_set():
            try:
                self.srv.settimeout(0.5)
                conn, _ = self.srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._handle_client, args=(conn,), daemon=True).start()

    def stop(self) -> None:
        self._stop.set()
        try:
            if self.srv: self.srv.close()
        except Exception:
            pass
        try:
            os.unlink(self.sock_path)
        except Exception:
            pass

    def _handle_client(self, conn: socket.socket) -> None:
        buf = b""
        try:
            while not self._stop.is_set():
                chunk = conn.recv(4096)
                if not chunk: break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip(): continue
                    resp = self._dispatch(line.decode("utf-8", errors="replace"))
                    # Compact form (no whitespace) matches proc_arm's output,
                    # so gateway's strstr-based "\"result\":" parser sees the
                    # value char immediately after the colon (e.g. '[' for
                    # arrays). Whitespace there breaks the parser.
                    conn.sendall((json.dumps(resp, separators=(",", ":")) + "\n").encode("utf-8"))
        except Exception as e:
            log("ERROR", f"client: {e}")
        finally:
            try: conn.close()
            except Exception: pass

    def _dispatch(self, line: str) -> Dict[str, Any]:
        try:
            msg = json.loads(line)
        except Exception as e:
            return {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": f"parse: {e}"}}
        rid = msg.get("id")
        method = msg.get("method", "")
        params = msg.get("params") or {}
        if not isinstance(params, dict):
            params = {"_args": params}
        fn = self.handlers.get(method)
        if not fn:
            return {"jsonrpc": "2.0", "id": rid, "error": {"code": -32601, "message": f"method not found: {method}"}}
        try:
            result = fn(**params)
            return {"jsonrpc": "2.0", "id": rid, "result": result}
        except TypeError as e:
            return {"jsonrpc": "2.0", "id": rid, "error": {"code": -32602, "message": f"params: {e}"}}
        except Exception as e:
            log("ERROR", f"{method}: {e}\n{traceback.format_exc()}")
            return {"jsonrpc": "2.0", "id": rid, "error": {"code": -32603, "message": str(e)}}


# ─── helpers used by dispatch lambdas ────────────────────────────────────
def _extract_joints(kw: Dict[str, Any]) -> List[float]:
    """Accept either {joint_1..joint_6: deg} or {angles: [..6 floats..]}."""
    if "angles" in kw and isinstance(kw["angles"], list) and len(kw["angles"]) == 6:
        return [float(a) for a in kw["angles"]]
    if "joints" in kw and isinstance(kw["joints"], list) and len(kw["joints"]) == 6:
        return [float(a) for a in kw["joints"]]
    out = []
    for i in range(1, 7):
        if f"joint_{i}" in kw: out.append(float(kw[f"joint_{i}"]))
        elif f"j{i}" in kw:    out.append(float(kw[f"j{i}"]))
        else:                  out.append(0.0)
    return out

def _filter_kw(kw: Dict[str, Any], allowed: Tuple[str, ...]) -> Dict[str, Any]:
    return {k: v for k, v in kw.items() if k in allowed}


# ─── main ────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default=DEFAULT_IFACE)
    ap.add_argument("--socket", default=DEFAULT_SOCKET)
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    args = ap.parse_args()

    ctrl = PiperController(args.iface, args.speed)
    srv = RpcServer(args.socket, ctrl)

    def _signal(signo, _frame):
        log("INFO", f"signal {signo} → shutting down")
        srv.stop()
        try: ctrl.shutdown()
        except Exception as e: log("ERROR", f"shutdown: {e}")
        sys.exit(0)
    signal.signal(signal.SIGINT,  _signal)
    signal.signal(signal.SIGTERM, _signal)

    try:
        ctrl.connect()
    except Exception as e:
        log("ERROR", f"connect failed: {e}\n{traceback.format_exc()}")
        sys.exit(1)

    log("INFO", "proc_piper ready")
    try:
        srv.serve_forever()
    finally:
        try: ctrl.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
