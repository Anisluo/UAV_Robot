#ifndef UAV_PROC_GRASP_APPLICATION_H
#define UAV_PROC_GRASP_APPLICATION_H

// Top-level entry for proc_grasp.
//
// proc_grasp is the "vision -> arm command" translation layer.
// It listens to UavCResult datagrams from proc_npu, picks the best
// detection, transforms it into the arm base frame via a hand-eye
// calibration, and drives proc_arm through its JSON-RPC socket.
//
// External surface is a JSON-RPC control socket (UAV_CTRL_PATH_D) that
// exposes grasp.start / grasp.stop / grasp.get_status / grasp.run_once
// / grasp.set_mode / grasp.set_hand_eye.
class GraspApplication {
public:
    int run();
};

#endif
