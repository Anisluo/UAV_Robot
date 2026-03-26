#include "npu_infer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <numeric>
#include <vector>

#include "rknn_api.h"

// ─── helpers ─────────────────────────────────────────────────────────────────

static inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// Inline RKNN error check
#define RKNN_CHECK(fn, ...)                                                 \
    do {                                                                     \
        int _r = (fn);                                                       \
        if (_r < 0) {                                                        \
            std::fprintf(stderr, "[npu_infer] " #fn " failed: %d\n", _r);   \
            return __VA_ARGS__;                                              \
        }                                                                    \
    } while (0)

// ─── ctor / dtor ─────────────────────────────────────────────────────────────

NpuInfer::NpuInfer() = default;

NpuInfer::~NpuInfer() {
    unload_model();
}

// ─── load_model ──────────────────────────────────────────────────────────────

bool NpuInfer::load_model(const std::string &model_path)
{
    unload_model();

    // Read model file into memory
    std::ifstream ifs(model_path, std::ios::binary | std::ios::ate);
    if (!ifs.is_open()) {
        std::fprintf(stderr, "[npu_infer] cannot open model: %s\n",
                     model_path.c_str());
        return false;
    }
    const auto model_size = static_cast<std::streamsize>(ifs.tellg());
    ifs.seekg(0);
    std::vector<uint8_t> model_data(static_cast<size_t>(model_size));
    ifs.read(reinterpret_cast<char *>(model_data.data()), model_size);
    ifs.close();

    // Init RKNN context
    rknn_context ctx = 0;
    RKNN_CHECK(rknn_init(&ctx, model_data.data(),
                         static_cast<uint32_t>(model_size), 0, nullptr),
               false);
    ctx_ = reinterpret_cast<void *>(ctx);

    // Query input / output count
    rknn_input_output_num io_num{};
    RKNN_CHECK(rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM,
                          &io_num, sizeof(io_num)), false);
    n_outputs_ = io_num.n_output;

    // Query first output shape to detect nc
    rknn_tensor_attr out_attr{};
    out_attr.index = 0;
    RKNN_CHECK(rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR,
                          &out_attr, sizeof(out_attr)), false);

    if (n_outputs_ == 3) {
        // Split-head format: [1, 64+nc, H, W] → nc = dims[1] - 64
        nc_ = static_cast<int>(out_attr.dims[1]) - DFL_BINS * 4;
    } else {
        // Single output: [1, 4+nc, 8400] → nc = dims[1] - 4
        nc_ = static_cast<int>(out_attr.dims[1]) - 4;
    }
    if (nc_ < 1) nc_ = 1;

    loaded_ = true;
    std::printf("[npu_infer] loaded %s | outputs=%u nc=%d\n",
                model_path.c_str(), n_outputs_, nc_);
    return true;
}

void NpuInfer::unload_model()
{
    if (ctx_) {
        rknn_destroy(reinterpret_cast<rknn_context>(ctx_));
        ctx_ = nullptr;
    }
    loaded_    = false;
    n_outputs_ = 0;
    nc_        = 1;
}

// ─── infer ───────────────────────────────────────────────────────────────────

bool NpuInfer::infer(const TensorInput &input, std::vector<RawDet> &dets) const
{
    if (!loaded_ || !ctx_) return false;

    rknn_context ctx = reinterpret_cast<rknn_context>(ctx_);

    // ── Set input ────────────────────────────────────────────────────────
    rknn_input inp[1]{};
    inp[0].index        = 0;
    inp[0].type         = RKNN_TENSOR_UINT8;
    inp[0].fmt          = RKNN_TENSOR_NHWC;
    inp[0].size         = input.width * input.height * 3U;
    inp[0].buf          = const_cast<uint8_t *>(input.data.data());
    inp[0].pass_through = 0; // let RKNN apply mean/std from config
    RKNN_CHECK(rknn_inputs_set(ctx, 1, inp), false);

    // ── Run ──────────────────────────────────────────────────────────────
    RKNN_CHECK(rknn_run(ctx, nullptr), false);

    // ── Get outputs ──────────────────────────────────────────────────────
    const uint32_t n = n_outputs_;
    std::vector<rknn_output> outputs(n);
    for (uint32_t i = 0; i < n; ++i) {
        outputs[i].index      = i;
        outputs[i].buf        = nullptr; // allocate internally
        outputs[i].size       = 0;
        outputs[i].is_prealloc= 0;
        outputs[i].want_float = 1;      // dequantize to float32
    }
    RKNN_CHECK(rknn_outputs_get(ctx, n, outputs.data(), nullptr), false);

    dets.clear();

    if (n_outputs_ == 3) {
        // ── 3-output split-head (DFL) ─────────────────────────────────
        // Strides for 640×640 input: head0=8, head1=16, head2=32
        static const int STRIDES[3] = {8, 16, 32};
        for (uint32_t i = 0; i < 3; ++i) {
            // Query this output's shape
            rknn_tensor_attr attr{};
            attr.index = i;
            rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
            // dims: [1, C, H, W]  (NCHW)
            const int C = static_cast<int>(attr.dims[1]);
            const int H = static_cast<int>(attr.dims[2]);
            const int W = static_cast<int>(attr.dims[3]);
            decode_head(static_cast<const float *>(outputs[i].buf),
                        C, H, W, STRIDES[i], nc_,
                        SCORE_PRE_THR, input, dets);
        }
    } else {
        // ── 1-output format [1, 4+nc, 8400] ──────────────────────────
        rknn_tensor_attr attr{};
        attr.index = 0;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
        const int channels = static_cast<int>(attr.dims[1]);
        const int anchors  = static_cast<int>(attr.dims[2]);
        decode_single(static_cast<const float *>(outputs[0].buf),
                      channels - 4, anchors,
                      SCORE_PRE_THR, input, dets);
    }

    rknn_outputs_release(ctx, n, outputs.data());

    // ── NMS ───────────────────────────────────────────────────────────────
    nms(dets, NMS_IOU_THR);

    return true;
}

// ─── decode_head (3-output DFL format) ───────────────────────────────────────

// DFL: softmax over `bins` values → expected value
float NpuInfer::dfl_integral(const float *p, int bins)
{
    // numerically stable softmax
    float max_v = p[0];
    for (int i = 1; i < bins; ++i) max_v = std::max(max_v, p[i]);

    float sum = 0.0f, wsum = 0.0f;
    for (int i = 0; i < bins; ++i) {
        float e = std::exp(p[i] - max_v);
        sum  += e;
        wsum += e * static_cast<float>(i);
    }
    return wsum / (sum + 1e-6f);
}
void NpuInfer::decode_head(const float *data,
                            // int channels, int grid_h, int grid_w,
                            int /*channels*/,int grid_h,int grid_w,
                            int stride, int nc, float score_thr,
                            const TensorInput &input,
                            std::vector<RawDet> &out) const
{
    // data layout: NCHW [1, channels, grid_h, grid_w]
    // channels = 4*DFL_BINS + nc
    const int hw = grid_h * grid_w;

    for (int h = 0; h < grid_h; ++h) {
        for (int w = 0; w < grid_w; ++w) {
            // ── Class scores ──────────────────────────────────────────────
            int best_cls = 0;
            float best_score = -1e9f;
            for (int c = 0; c < nc; ++c) {
                const int ch = DFL_BINS * 4 + c;
                float s = sigmoid(data[ch * hw + h * grid_w + w]);
                if (s > best_score) { best_score = s; best_cls = c; }
            }
            if (best_score < score_thr) continue;

            // ── DFL regression → ltrb (in model pixels) ──────────────────
            float reg[4];
            for (int k = 0; k < 4; ++k) {
                float bins_buf[DFL_BINS];
                for (int b = 0; b < DFL_BINS; ++b) {
                    const int ch = k * DFL_BINS + b;
                    bins_buf[b] = data[ch * hw + h * grid_w + w];
                }
                reg[k] = dfl_integral(bins_buf, DFL_BINS);
            }

            const float anchor_x = (static_cast<float>(w) + 0.5f)
                                    * static_cast<float>(stride);
            const float anchor_y = (static_cast<float>(h) + 0.5f)
                                    * static_cast<float>(stride);

            // ltrb → xyxy in model space
            const float mx1 = anchor_x - reg[0] * static_cast<float>(stride);
            const float my1 = anchor_y - reg[1] * static_cast<float>(stride);
            const float mx2 = anchor_x + reg[2] * static_cast<float>(stride);
            const float my2 = anchor_y + reg[3] * static_cast<float>(stride);

            // ── Undo letterbox → original image coords ────────────────────
            const float ix1 = (mx1 - input.lb_pad_x) / input.lb_scale;
            const float iy1 = (my1 - input.lb_pad_y) / input.lb_scale;
            const float ix2 = (mx2 - input.lb_pad_x) / input.lb_scale;
            const float iy2 = (my2 - input.lb_pad_y) / input.lb_scale;

            // Clamp to image bounds
            const float iw = static_cast<float>(input.orig_width  - 1);
            const float ih = static_cast<float>(input.orig_height - 1);
            RawDet d;
            d.class_id = best_cls;
            d.score    = best_score;
            d.x1 = std::max(0.f, std::min(ix1, iw));
            d.y1 = std::max(0.f, std::min(iy1, ih));
            d.x2 = std::max(0.f, std::min(ix2, iw));
            d.y2 = std::max(0.f, std::min(iy2, ih));
            out.push_back(d);
        }
    }
}

// ─── decode_single (1-output format [1, 4+nc, 8400]) ─────────────────────────

void NpuInfer::decode_single(const float *data, int nc, int anchors,
                              float score_thr,
                              const TensorInput &input,
                              std::vector<RawDet> &out) const
{
    // data: [4+nc, anchors] in C-major order
    // [0..3, i]  = xc, yc, w, h  (model pixels 0-640)
    // [4..,  i]  = class scores (already sigmoid-ed by ultralytics export)
    for (int i = 0; i < anchors; ++i) {
        int best_cls = 0;
        float best_score = -1e9f;
        for (int c = 0; c < nc; ++c) {
            float s = data[(4 + c) * anchors + i];
            if (s > best_score) { best_score = s; best_cls = c; }
        }
        if (best_score < score_thr) continue;

        const float xc = data[0 * anchors + i];
        const float yc = data[1 * anchors + i];
        const float bw = data[2 * anchors + i];
        const float bh = data[3 * anchors + i];

        const float mx1 = xc - bw * 0.5f;
        const float my1 = yc - bh * 0.5f;
        const float mx2 = xc + bw * 0.5f;
        const float my2 = yc + bh * 0.5f;

        const float ix1 = (mx1 - input.lb_pad_x) / input.lb_scale;
        const float iy1 = (my1 - input.lb_pad_y) / input.lb_scale;
        const float ix2 = (mx2 - input.lb_pad_x) / input.lb_scale;
        const float iy2 = (my2 - input.lb_pad_y) / input.lb_scale;

        const float iw = static_cast<float>(input.orig_width  - 1);
        const float ih = static_cast<float>(input.orig_height - 1);
        RawDet d;
        d.class_id = best_cls;
        d.score    = best_score;
        d.x1 = std::max(0.f, std::min(ix1, iw));
        d.y1 = std::max(0.f, std::min(iy1, ih));
        d.x2 = std::max(0.f, std::min(ix2, iw));
        d.y2 = std::max(0.f, std::min(iy2, ih));
        out.push_back(d);
    }
}

// ─── NMS ─────────────────────────────────────────────────────────────────────

float NpuInfer::iou(const RawDet &a, const RawDet &b)
{
    const float ix1 = std::max(a.x1, b.x1);
    const float iy1 = std::max(a.y1, b.y1);
    const float ix2 = std::min(a.x2, b.x2);
    const float iy2 = std::min(a.y2, b.y2);
    const float inter_w = std::max(0.f, ix2 - ix1);
    const float inter_h = std::max(0.f, iy2 - iy1);
    const float inter   = inter_w * inter_h;
    if (inter <= 0.f) return 0.f;
    const float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    const float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

void NpuInfer::nms(std::vector<RawDet> &dets, float iou_thr)
{
    // Sort by score descending
    std::sort(dets.begin(), dets.end(),
              [](const RawDet &a, const RawDet &b){ return a.score > b.score; });

    std::vector<bool> suppressed(dets.size(), false);
    std::vector<RawDet> kept;

    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        kept.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (suppressed[j]) continue;
            if (dets[i].class_id == dets[j].class_id &&
                iou(dets[i], dets[j]) > iou_thr) {
                suppressed[j] = true;
            }
        }
    }

    dets = std::move(kept);
}
