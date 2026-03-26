#ifndef UAV_PROC_NPU_INFER_H
#define UAV_PROC_NPU_INFER_H

#include <cstdint>
#include <string>
#include <vector>

#include "preprocess.h"

// Raw detection in original image pixel coordinates (before depth projection).
struct RawDet {
    int   class_id;
    float score;
    float x1;   // top-left     (original image pixels)
    float y1;
    float x2;   // bottom-right
    float y2;
};

// ─────────────────────────────────────────────────────────────────────────────
// NpuInfer  –  wraps RKNN context and runs YOLOv8 inference.
//
// Supported RKNN model formats (auto-detected from n_outputs):
//   • 3 outputs: [1, 64+nc, 80,80] [1,64+nc,40,40] [1,64+nc,20,20]
//     (Rockchip model-zoo / export_onnx.py split-head format, recommended)
//   • 1 output:  [1, 4+nc, 8400]
//     (standard ultralytics onnx → rknn, no head split)
// ─────────────────────────────────────────────────────────────────────────────
class NpuInfer {
public:
    NpuInfer();
    ~NpuInfer();

    bool load_model(const std::string &model_path);
    void unload_model();
    bool loaded() const { return loaded_; }

    // Returns raw detections in original image space (letterbox already undone).
    bool infer(const TensorInput &input, std::vector<RawDet> &dets) const;

private:
    // DFL decode for one regression bin-vector of length DFL_BINS.
    static float dfl_integral(const float *p, int bins);

    // Greedy NMS; modifies dets in-place.
    static void nms(std::vector<RawDet> &dets, float iou_thr);
    static float iou(const RawDet &a, const RawDet &b);

    // Decode a single split-head output tensor (3-output format).
    void decode_head(const float *data, int channels, int grid_h, int grid_w,
                     int stride, int nc, float score_thr,
                     const TensorInput &input,
                     std::vector<RawDet> &out) const;

    // Decode single-output format [1, 4+nc, 8400].
    void decode_single(const float *data, int nc, int anchors,
                       float score_thr,
                       const TensorInput &input,
                       std::vector<RawDet> &out) const;

    bool     loaded_   = false;
    void    *ctx_      = nullptr;   // rknn_context (void* avoids including rknn_api.h here)
    uint32_t n_outputs_= 0;
    int      nc_       = 1;         // number of classes (detected from model)

    static constexpr int   DFL_BINS      = 16;
    static constexpr float SCORE_PRE_THR = 0.25f;  // pre-NMS threshold
    static constexpr float NMS_IOU_THR   = 0.45f;
};

#endif // UAV_PROC_NPU_INFER_H
