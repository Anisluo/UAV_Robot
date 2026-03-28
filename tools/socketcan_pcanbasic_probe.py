#!/usr/bin/env python3
import argparse
import importlib.util
import json
import os
import socket
import struct
import sys
import time
import types
from ctypes import Structure, c_ubyte, c_uint, c_ushort


TPCANHandle = c_ushort
TPCANStatus = int
TPCANMessageType = c_ubyte
TPCANBaudrate = c_ushort

PCAN_ERROR_OK = TPCANStatus(0x00000)
PCAN_ERROR_QRCVEMPTY = TPCANStatus(0x00020)
PCAN_ERROR_ILLPARAMVAL = TPCANStatus(0x08000)

PCAN_NONEBUS = TPCANHandle(0x00)
PCAN_USBBUS1 = TPCANHandle(0x51)
PCAN_USBBUS2 = TPCANHandle(0x52)
PCAN_USBBUS3 = TPCANHandle(0x53)
PCAN_USBBUS4 = TPCANHandle(0x54)
PCAN_USBBUS5 = TPCANHandle(0x55)
PCAN_USBBUS6 = TPCANHandle(0x56)

PCAN_BAUD_500K = TPCANBaudrate(0x001C)
PCAN_MESSAGE_STANDARD = TPCANMessageType(0x00)
PCAN_MESSAGE_EXTENDED = TPCANMessageType(0x02)

_CAN_EFF_FLAG = 0x80000000
_CAN_EFF_MASK = 0x1FFFFFFF


def _scalar(value):
    raw = getattr(value, "value", value)
    if isinstance(raw, (bytes, bytearray)):
        return int.from_bytes(raw, byteorder="little")
    return int(raw)


class TPCANMsg(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("MSGTYPE", TPCANMessageType),
        ("LEN", c_ubyte),
        ("DATA", c_ubyte * 8),
    ]


class TPCANTimestamp(Structure):
    _fields_ = [
        ("millis", c_uint),
        ("millis_overflow", c_ushort),
        ("micros", c_ushort),
    ]


class SocketCANPCANBasic:
    def __init__(self, iface: str):
        self._iface = iface
        self._sock = None

    def _ensure_socket(self):
        if self._sock is not None:
            return
        self._sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((self._iface,))
        self._sock.settimeout(0.05)

    def Initialize(self, Channel, Btr0Btr1, *_args):
        del Channel, Btr0Btr1
        self._ensure_socket()
        return PCAN_ERROR_OK

    def InitializeFD(self, Channel, BitrateFD):
        del BitrateFD
        return self.Initialize(Channel, None)

    def Uninitialize(self, Channel):
        del Channel
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        return PCAN_ERROR_OK

    def Reset(self, Channel):
        del Channel
        return PCAN_ERROR_OK

    def GetStatus(self, Channel):
        del Channel
        return PCAN_ERROR_OK

    def SetValue(self, Channel, Parameter, Buffer):
        del Channel, Parameter, Buffer
        return PCAN_ERROR_OK

    def GetValue(self, Channel, Parameter):
        del Channel, Parameter
        return PCAN_ERROR_OK, 0

    def GetErrorText(self, Error, Language=0):
        del Language
        return PCAN_ERROR_OK, f"socketcan shim error=0x{int(Error):x}"

    def Write(self, Channel, MessageBuffer):
        del Channel
        self._ensure_socket()
        can_id = _scalar(MessageBuffer.ID)
        msgtype = _scalar(MessageBuffer.MSGTYPE)
        if msgtype & _scalar(PCAN_MESSAGE_EXTENDED):
            can_id = (can_id & _CAN_EFF_MASK) | _CAN_EFF_FLAG
        dlc = _scalar(MessageBuffer.LEN)
        data = bytes(MessageBuffer.DATA[:dlc])
        frame = struct.pack("=IB3x8s", can_id, dlc, data.ljust(8, b"\x00"))
        self._sock.send(frame)
        return PCAN_ERROR_OK

    def Read(self, Channel):
        del Channel
        self._ensure_socket()
        msg = TPCANMsg()
        ts = TPCANTimestamp()
        try:
            frame = self._sock.recv(16)
        except socket.timeout:
            return PCAN_ERROR_QRCVEMPTY, msg, ts
        can_id, can_dlc, data = struct.unpack("=IB3x8s", frame)
        if can_id & _CAN_EFF_FLAG:
            msg.ID = can_id & _CAN_EFF_MASK
            msg.MSGTYPE = PCAN_MESSAGE_EXTENDED
        else:
            msg.ID = can_id & 0x7FF
            msg.MSGTYPE = PCAN_MESSAGE_STANDARD
        msg.LEN = can_dlc
        for i, b in enumerate(data[:can_dlc]):
            msg.DATA[i] = b
        now = time.time()
        millis = int(now * 1000.0)
        ts.millis = millis & 0xFFFFFFFF
        ts.millis_overflow = (millis >> 32) & 0xFFFF
        ts.micros = int((now * 1000000.0) % 1000)
        return PCAN_ERROR_OK, msg, ts


def install_fake_modules(iface: str):
    fake = types.ModuleType("PCANBasic")
    fake.TPCANHandle = TPCANHandle
    fake.TPCANStatus = TPCANStatus
    fake.TPCANMessageType = TPCANMessageType
    fake.TPCANBaudrate = TPCANBaudrate
    fake.TPCANMsg = TPCANMsg
    fake.TPCANTimestamp = TPCANTimestamp
    fake.PCAN_ERROR_OK = PCAN_ERROR_OK
    fake.PCAN_ERROR_QRCVEMPTY = PCAN_ERROR_QRCVEMPTY
    fake.PCAN_ERROR_ILLPARAMVAL = PCAN_ERROR_ILLPARAMVAL
    fake.PCAN_NONEBUS = PCAN_NONEBUS
    fake.PCAN_USBBUS1 = PCAN_USBBUS1
    fake.PCAN_USBBUS2 = PCAN_USBBUS2
    fake.PCAN_USBBUS3 = PCAN_USBBUS3
    fake.PCAN_USBBUS4 = PCAN_USBBUS4
    fake.PCAN_USBBUS5 = PCAN_USBBUS5
    fake.PCAN_USBBUS6 = PCAN_USBBUS6
    fake.PCAN_BAUD_500K = PCAN_BAUD_500K
    fake.PCAN_MESSAGE_STANDARD = PCAN_MESSAGE_STANDARD
    fake.PCAN_MESSAGE_EXTENDED = PCAN_MESSAGE_EXTENDED
    fake.PCANBasic = lambda: SocketCANPCANBasic(iface)
    sys.modules["PCANBasic"] = fake

    rtb = types.ModuleType("roboticstoolbox")

    class DHRobot:
        def __init__(self, *args, **kwargs):
            del args, kwargs

    class RevoluteDH:
        def __init__(self, *args, **kwargs):
            del args, kwargs

    rtb.DHRobot = DHRobot
    rtb.RevoluteDH = RevoluteDH
    sys.modules["roboticstoolbox"] = rtb
    sys.modules["spatialmath"] = types.ModuleType("spatialmath")
    robodk = types.ModuleType("robodk")
    robolink = types.ModuleType("robolink")
    robodk.robolink = robolink
    sys.modules["robodk"] = robodk
    sys.modules["robolink"] = robolink


def load_controller_core(so_path: str):
    spec = importlib.util.spec_from_file_location("controller_core", so_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load controller_core from {so_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def normalize_angles(values):
    out = []
    for value in values:
        if value is None:
            out.append(0.0)
        else:
            out.append(float(value))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can4")
    ap.add_argument("--sdk-so", required=True)
    ap.add_argument("--usb-id", type=int, default=1)
    ap.add_argument("--action", choices=["angles", "home"], default="angles")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    try:
        sdk_dir = os.path.dirname(os.path.abspath(args.sdk_so))
        if sdk_dir not in sys.path:
            sys.path.insert(0, sdk_dir)
        install_fake_modules(args.iface)
        controller_core = load_controller_core(args.sdk_so)
        mc = controller_core.MotorControl(usb_id=args.usb_id, use_robodk=False)
        initialized = bool(getattr(mc, "initialized", False))
        if args.action == "home":
            home_result = mc.sequential_home_all()
            angles = normalize_angles(mc.read_motor_angles())
        else:
            home_result = None
            angles = normalize_angles(mc.read_motor_angles())
        mc.closeCAN()
        if args.json:
            payload = {"ok": True, "initialized": initialized, "action": args.action, "angles": angles}
            if home_result is not None:
                payload["home_result"] = home_result
            print(json.dumps(payload))
        else:
            print("initialized:", initialized)
            if home_result is not None:
                print("home_result:", home_result)
            print("angles:", angles)
    except Exception as exc:
        if args.json:
            print(json.dumps({"ok": False, "error": str(exc)}))
        else:
            raise


if __name__ == "__main__":
    main()
