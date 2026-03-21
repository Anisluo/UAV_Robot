#include "rs_capture.h"

#include <chrono>
#include <cstdio>

namespace {
uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
} // namespace

RSCapture::RSCapture()
    : next_frame_id_(1),
      width_(640),
      height_(480),
      fps_(30),
      exposure_us_(0),
      running_(false) {}

bool RSCapture::start() {
    if (running_) {
        return true;
    }
    try {
        cfg_.enable_stream(RS2_STREAM_COLOR, static_cast<int>(width_),
                           static_cast<int>(height_), RS2_FORMAT_BGR8,
                           static_cast<int>(fps_));
        cfg_.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(width_),
                           static_cast<int>(height_), RS2_FORMAT_Z16,
                           static_cast<int>(fps_));
        pipe_.start(cfg_);
        running_ = true;
    } catch (const rs2::error &e) {
        std::fprintf(stderr, "proc_realsense: rs2 start failed: %s\n", e.what());
        return false;
    }
    return true;
}

void RSCapture::stop() {
    if (!running_) {
        return;
    }
    try {
        pipe_.stop();
    } catch (...) {}
    running_ = false;
}

void RSCapture::set_profile(uint32_t width, uint32_t height, uint32_t fps) {
    width_  = width;
    height_ = height;
    fps_    = fps == 0 ? 30 : fps;

    if (running_) {
        stop();
        cfg_ = rs2::config{};
        start();
    }
}

void RSCapture::set_exposure(int32_t exposure_us) {
    exposure_us_ = exposure_us;
    if (!running_) {
        return;
    }
    try {
        auto sensors = pipe_.get_active_profile().get_device().query_sensors();
        for (auto &sensor : sensors) {
            if (sensor.is<rs2::color_sensor>()) {
                if (exposure_us <= 0) {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0F);
                } else {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0F);
                    sensor.set_option(RS2_OPTION_EXPOSURE,
                                      static_cast<float>(exposure_us));
                }
                break;
            }
        }
    } catch (const rs2::error &e) {
        std::fprintf(stderr, "proc_realsense: set_exposure failed: %s\n", e.what());
    }
}

bool RSCapture::snapshot(CaptureFrame &frame) {
    if (!running_) {
        return false;
    }

    rs2::frameset fs;
    try {
        fs = pipe_.wait_for_frames(3000 /* ms */);
    } catch (const rs2::error &e) {
        std::fprintf(stderr, "proc_realsense: wait_for_frames failed: %s — restarting pipeline\n", e.what());
        stop();
        cfg_ = rs2::config{};
        if (!start()) {
            return false;
        }
        try {
            fs = pipe_.wait_for_frames(5000 /* ms */);
        } catch (const rs2::error &e2) {
            std::fprintf(stderr, "proc_realsense: restart failed: %s\n", e2.what());
            return false;
        }
    }

    rs2::video_frame color_frame = fs.get_color_frame();
    rs2::depth_frame depth_frame = fs.get_depth_frame();
    if (!color_frame || !depth_frame) {
        return false;
    }

    const uint32_t w = static_cast<uint32_t>(color_frame.get_width());
    const uint32_t h = static_cast<uint32_t>(color_frame.get_height());
    const uint32_t color_stride = w * 3U;
    const uint32_t depth_stride = w * 2U;

    // intrinsics from color stream
    rs2_intrinsics intr = color_frame.get_profile()
                              .as<rs2::video_stream_profile>()
                              .get_intrinsics();

    // depth scale from depth sensor
    float depth_scale = 0.001F;
    try {
        auto sensors = pipe_.get_active_profile().get_device().query_sensors();
        for (auto &sensor : sensors) {
            if (sensor.is<rs2::depth_sensor>()) {
                depth_scale = sensor.as<rs2::depth_sensor>().get_depth_scale();
                break;
            }
        }
    } catch (...) {}

    frame.frame_id     = next_frame_id_++;
    frame.timestamp_ns = now_ns();
    frame.width        = w;
    frame.height       = h;
    frame.stride       = color_stride;
    frame.fx           = intr.fx;
    frame.fy           = intr.fy;
    frame.cx           = intr.ppx;
    frame.cy           = intr.ppy;
    frame.depth_scale  = depth_scale;

    const auto *src_color = static_cast<const uint8_t *>(color_frame.get_data());
    frame.color.assign(src_color,
                       src_color + static_cast<size_t>(color_stride) * h);

    const auto *src_depth = static_cast<const uint8_t *>(depth_frame.get_data());
    frame.depth.assign(src_depth,
                       src_depth + static_cast<size_t>(depth_stride) * h);

    return true;
}
