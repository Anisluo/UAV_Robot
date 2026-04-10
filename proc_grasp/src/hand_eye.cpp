#include "hand_eye.h"

#include <cmath>

HandEye::HandEye()
    : m_{{1.0F, 0.0F, 0.0F, 0.0F,
          0.0F, 1.0F, 0.0F, 0.0F,
          0.0F, 0.0F, 1.0F, 0.0F}} {}

bool HandEye::set(const std::array<float, 12> &m)
{
    for (float v : m) {
        if (std::isnan(v) || std::isinf(v)) {
            return false;
        }
    }
    m_ = m;
    return true;
}

void HandEye::transform_point(float xc, float yc, float zc,
                              float *xb, float *yb, float *zb) const
{
    if (xb != nullptr) {
        *xb = m_[0] * xc + m_[1] * yc + m_[2]  * zc + m_[3];
    }
    if (yb != nullptr) {
        *yb = m_[4] * xc + m_[5] * yc + m_[6]  * zc + m_[7];
    }
    if (zb != nullptr) {
        *zb = m_[8] * xc + m_[9] * yc + m_[10] * zc + m_[11];
    }
}
