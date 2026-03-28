#ifndef UAV_PROC_REALSENSE_RS_CAPTURE_H
#define UAV_PROC_REALSENSE_RS_CAPTURE_H

#include <cstdint>
#include <vector>

#include <librealsense2/rs.hpp>

struct CaptureFrame {
    uint64_t frame_id;
    uint64_t timestamp_ns;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    float fx;
    float fy;
    float cx;
    float cy;
    float depth_scale;
    std::vector<uint8_t> color;
    std::vector<uint8_t> depth;
};

class RSCapture {
public:
    RSCapture();
    bool start();
    void stop();
    void set_profile(uint32_t width, uint32_t height, uint32_t fps);
    void set_exposure(int32_t exposure_us);
    bool snapshot(CaptureFrame &frame);

private:
    bool apply_profile(bool enable_depth);
    bool try_start_mode(bool enable_depth, const char *label);

    uint64_t next_frame_id_;
    uint32_t width_;
    uint32_t height_;
    uint32_t fps_;
    int32_t exposure_us_;
    bool running_;
    bool depth_enabled_;
    rs2::pipeline pipe_;
    rs2::config cfg_;
};

#endif
