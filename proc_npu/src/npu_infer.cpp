#include "npu_infer.h"

NpuInfer::NpuInfer() : loaded_(false), model_path_() {}

bool NpuInfer::load_model(const std::string &model_path) {
    model_path_ = model_path;
    loaded_ = true;
    return true;
}

void NpuInfer::unload_model() {
    loaded_ = false;
    model_path_.clear();
}

bool NpuInfer::loaded() const {
    return loaded_;
}

bool NpuInfer::infer(const TensorInput &input, std::vector<RawDet> &dets) const {
    if (!loaded_) {
        return false;
    }
    RawDet d{};
    d.class_id = 1;
    d.score = 0.92F;
    d.x1 = 0.20F * static_cast<float>(input.width);
    d.y1 = 0.20F * static_cast<float>(input.height);
    d.x2 = 0.60F * static_cast<float>(input.width);
    d.y2 = 0.70F * static_cast<float>(input.height);
    dets.clear();
    dets.push_back(d);
    return true;
}
