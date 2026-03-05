#include "rs_capture.h"

#include <chrono>
#include <thread>

namespace {
uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
}

RSCapture::RSCapture()
    : next_frame_id_(1),
      width_(640),
      height_(480),
      fps_(30),
      exposure_us_(0) {}

bool RSCapture::start() {
    return true;
}

void RSCapture::stop() {}

void RSCapture::set_profile(uint32_t width, uint32_t height, uint32_t fps) {
    width_ = width;
    height_ = height;
    fps_ = fps == 0 ? 30 : fps;
}

void RSCapture::set_exposure(int32_t exposure_us) {
    exposure_us_ = exposure_us;
    (void)exposure_us_;
}

bool RSCapture::snapshot(CaptureFrame &frame) {
    const uint32_t color_stride = width_ * 3U;
    const uint32_t depth_stride = width_ * 2U;
    frame.frame_id = next_frame_id_++;
    frame.timestamp_ns = now_ns();
    frame.width = width_;
    frame.height = height_;
    frame.stride = color_stride;
    frame.fx = 615.0F;
    frame.fy = 615.0F;
    frame.cx = static_cast<float>(width_) / 2.0F;
    frame.cy = static_cast<float>(height_) / 2.0F;
    frame.depth_scale = 0.001F;
    frame.color.assign(static_cast<size_t>(color_stride) * height_, 0U);
    frame.depth.assign(static_cast<size_t>(depth_stride) * height_, 0U);

    for (size_t i = 0; i < frame.color.size(); i += 3) {
        frame.color[i] = static_cast<uint8_t>((frame.frame_id + i) & 0xFFU);
        frame.color[i + 1] = static_cast<uint8_t>((i / 3U) & 0xFFU);
        frame.color[i + 2] = 128U;
    }

    for (size_t i = 0; i + 1 < frame.depth.size(); i += 2) {
        const uint16_t mm = static_cast<uint16_t>((1000U + (frame.frame_id % 200U)));
        frame.depth[i] = static_cast<uint8_t>(mm & 0xFFU);
        frame.depth[i + 1] = static_cast<uint8_t>((mm >> 8) & 0xFFU);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000U / fps_));
    return true;
}
