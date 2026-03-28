#include "rs_capture.h"

#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <thread>

namespace {
uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

constexpr int kFrameTimeoutMs = 1500;
constexpr int kWarmupAttempts = 8;
constexpr int kWarmupDrops = 2;
constexpr int kRecoveryAttempts = 3;
constexpr int kStartAttempts = 3;

bool wait_for_valid_frameset(rs2::pipeline &pipe,
                             rs2::frameset &out,
                             int timeout_ms,
                             int attempts,
                             int warmup_drops,
                             bool require_depth,
                             const char *stage) {
    int drops_left = warmup_drops;
    for (int i = 0; i < attempts; ++i) {
        try {
            rs2::frameset fs = pipe.wait_for_frames(timeout_ms);
            rs2::video_frame color_frame = fs.get_color_frame();
            rs2::depth_frame depth_frame = fs.get_depth_frame();
            const bool has_color = static_cast<bool>(color_frame);
            const bool has_depth = static_cast<bool>(depth_frame);
            if (!has_color || (require_depth && !has_depth)) {
                std::fprintf(stderr,
                             "proc_realsense: %s frame missing color=%d depth=%d attempt=%d/%d\n",
                             stage,
                             has_color ? 1 : 0,
                             has_depth ? 1 : 0,
                             i + 1,
                             attempts);
            } else if (drops_left > 0) {
                --drops_left;
            } else {
                out = fs;
                return true;
            }
        } catch (const rs2::error &e) {
            std::fprintf(stderr,
                         "proc_realsense: %s wait_for_frames failed: %s attempt=%d/%d\n",
                         stage,
                         e.what(),
                         i + 1,
                         attempts);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
    return false;
}
} // namespace

RSCapture::RSCapture()
    : next_frame_id_(1),
      width_(640),
      height_(480),
      fps_(30),
      exposure_us_(0),
      running_(false),
      depth_enabled_(true) {}

bool RSCapture::apply_profile(bool enable_depth) {
    cfg_ = rs2::config{};
    cfg_.enable_stream(RS2_STREAM_COLOR,
                       static_cast<int>(width_),
                       static_cast<int>(height_),
                       RS2_FORMAT_BGR8,
                       static_cast<int>(fps_));
    if (enable_depth) {
        cfg_.enable_stream(RS2_STREAM_DEPTH,
                           static_cast<int>(width_),
                           static_cast<int>(height_),
                           RS2_FORMAT_Z16,
                           static_cast<int>(fps_));
    }
    return true;
}

bool RSCapture::try_start_mode(bool enable_depth, const char *label) {
    for (int attempt = 1; attempt <= kStartAttempts; ++attempt) {
        try {
            apply_profile(enable_depth);
            pipe_.start(cfg_);
            running_ = true;
            depth_enabled_ = enable_depth;
            if (exposure_us_ != 0) {
                set_exposure(exposure_us_);
            }
            rs2::frameset warmup;
            if (!wait_for_valid_frameset(pipe_,
                                         warmup,
                                         kFrameTimeoutMs,
                                         kWarmupAttempts,
                                         kWarmupDrops,
                                         enable_depth,
                                         label)) {
                std::fprintf(stderr,
                             "proc_realsense: %s warmup failed width=%u height=%u fps=%u depth=%d attempt=%d/%d\n",
                             label,
                             width_,
                             height_,
                             fps_,
                             enable_depth ? 1 : 0,
                             attempt,
                             kStartAttempts);
                stop();
            } else {
                return true;
            }
        } catch (const rs2::error &e) {
            std::fprintf(stderr,
                         "proc_realsense: %s start failed depth=%d attempt=%d/%d: %s\n",
                         label,
                         enable_depth ? 1 : 0,
                         attempt,
                         kStartAttempts,
                         e.what());
            stop();
        }

        if (attempt < kStartAttempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }
    return false;
}

bool RSCapture::start() {
    if (running_) {
        return true;
    }

    const char *disable_depth_raw = std::getenv("UAV_REALSENSE_DISABLE_DEPTH");
    const bool prefer_color_only =
        (disable_depth_raw != nullptr &&
         disable_depth_raw[0] != '\0' &&
         disable_depth_raw[0] != '0');

    if (prefer_color_only) {
        if (try_start_mode(false, "start-color")) {
            return true;
        }
        return false;
    }

    if (try_start_mode(true, "start-depth")) {
        return true;
    }
    std::fprintf(stderr,
                 "proc_realsense: depth mode unavailable, falling back to color-only\n");
    return try_start_mode(false, "start-color");
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
    if (!wait_for_valid_frameset(pipe_,
                                 fs,
                                 kFrameTimeoutMs,
                                 kRecoveryAttempts,
                                 0,
                                 depth_enabled_,
                                 "snapshot")) {
        std::fprintf(stderr,
                     "proc_realsense: snapshot failed — restarting pipeline width=%u height=%u fps=%u\n",
                     width_,
                     height_,
                     fps_);
        stop();
        if (!start()) {
            return false;
        }
        if (!wait_for_valid_frameset(pipe_,
                                     fs,
                                     kFrameTimeoutMs,
                                     kRecoveryAttempts,
                                     0,
                                     depth_enabled_,
                                     "recover")) {
            std::fprintf(stderr, "proc_realsense: recover wait_for_frames failed\n");
            return false;
        }
    }

    rs2::video_frame color_frame = fs.get_color_frame();
    rs2::depth_frame depth_frame = fs.get_depth_frame();
    if (!color_frame || (depth_enabled_ && !depth_frame)) {
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
    if (depth_enabled_) {
        try {
            auto sensors = pipe_.get_active_profile().get_device().query_sensors();
            for (auto &sensor : sensors) {
                if (sensor.is<rs2::depth_sensor>()) {
                    depth_scale = sensor.as<rs2::depth_sensor>().get_depth_scale();
                    break;
                }
            }
        } catch (...) {}
    }

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

    if (depth_enabled_ && depth_frame) {
        const auto *src_depth = static_cast<const uint8_t *>(depth_frame.get_data());
        frame.depth.assign(src_depth,
                           src_depth + static_cast<size_t>(depth_stride) * h);
    } else {
        frame.depth.assign(static_cast<size_t>(depth_stride) * h, 0);
        frame.depth_scale = 0.0F;
    }

    return true;
}
