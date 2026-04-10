#ifndef UAV_PROC_GRASP_HAND_EYE_H
#define UAV_PROC_GRASP_HAND_EYE_H

#include <array>

// A minimal hand-eye (camera -> arm base) transform.
//
// Stored as a 3x4 homogeneous matrix flattened row-major (12 floats):
//   [ r00 r01 r02 tx
//     r10 r11 r12 ty
//     r20 r21 r22 tz ]
//
// Defaults to identity — proc_grasp will pass camera-frame coordinates
// straight through to the arm until a real calibration is pushed via
// grasp.set_hand_eye. All angles are forwarded unchanged; full rotation
// composition can be layered on later once real calibration data lands.
class HandEye {
public:
    HandEye();

    // Replace the 3x4 affine. Returns false if any NaN/Inf is present.
    bool set(const std::array<float, 12> &m);

    // Current 3x4 affine snapshot.
    std::array<float, 12> get() const { return m_; }

    // Transform a 3D point (mm) from camera frame to base frame.
    void transform_point(float xc, float yc, float zc,
                         float *xb, float *yb, float *zb) const;

private:
    std::array<float, 12> m_;
};

#endif
