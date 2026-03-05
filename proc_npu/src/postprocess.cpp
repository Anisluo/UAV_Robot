#include "postprocess.h"

UavCResult Postprocess::run(const InferenceFrame &frame, const std::vector<RawDet> &raw, float threshold) const {
    UavCResult out{};
    out.frame_id = frame.slot.frame_id;
    out.num_detections = 0;

    for (const RawDet &d : raw) {
        if (out.num_detections >= UAV_MAX_DETECTIONS) {
            break;
        }
        if (d.score < threshold) {
            continue;
        }
        UavDetection det{};
        det.class_id = d.class_id;
        det.score = d.score;
        det.x1 = d.x1;
        det.y1 = d.y1;
        det.x2 = d.x2;
        det.y2 = d.y2;
        det.has_xyz = 0;
        out.detections[out.num_detections++] = det;
    }

    return out;
}
