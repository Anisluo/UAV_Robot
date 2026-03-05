#ifndef UAV_PROC_NPU_INFER_H
#define UAV_PROC_NPU_INFER_H

#include <string>
#include <vector>

#include "preprocess.h"

struct RawDet {
    int class_id;
    float score;
    float x1;
    float y1;
    float x2;
    float y2;
};

class NpuInfer {
public:
    NpuInfer();
    bool load_model(const std::string &model_path);
    void unload_model();
    bool loaded() const;
    bool infer(const TensorInput &input, std::vector<RawDet> &dets) const;

private:
    bool loaded_;
    std::string model_path_;
};

#endif
