#pragma once

#include <vector>

#include "ret_codes.h"
#include "Logger.h"


#define OBJ_NAME_MAX_SIZE 16
#define OBJ_NUM_MAX_SIZE 64
#define OBJ_CLASS_NUM     3
#define PROP_BOX_SIZE     (5 + OBJ_CLASS_NUM)

namespace vision {

typedef struct {
    unsigned int width;
    unsigned int height;
} Resolution;

typedef struct Attitude {
    float roll;
    float pitch;
    int yaw;
    bool operator==(const struct Attitude& other) const {
        return (std::abs(roll - other.roll) < 0.1f) && 
               (std::abs(pitch - other.pitch) < 0.1f) && 
               (yaw == other.yaw);
    }
    bool operator!=(const Attitude& other) const {
        return !(*this == other);
    }
} Attitude;

typedef struct {
    int left; // x1
    int right; // x2
    int top; // y1
    int bottom; // y2
} BoxRect;

typedef struct {
    float left;
    float right;
    float top;
    float bottom;
} NormalizedBoxRect;

typedef struct DetectedObject {
    DetectedObject(): obj_class(0), confidence(0.0f), track_id(-1), box({0, 0, 0, 0}) {}
    ~DetectedObject() {}
    // char name[OBJ_NAME_MAX_SIZE];
    int obj_class;
    BoxRect box;
    NormalizedBoxRect nbox;
    float confidence;
    int track_id;
    friend std::ostream& operator<<(std::ostream& os, const DetectedObject& obj) {
        os << "detected obj=" << obj.obj_class 
           << ", track_id=" << obj.track_id
           << ", conf=" << obj.confidence 
           << ", coords=[(" << obj.box.left << ", " << obj.box.top 
           << "), (" << obj.box.right << ", " << obj.box.bottom << ")]";
        return os;
    }
    void Reset() {
        obj_class = 0;
        confidence = 0.0f;
        track_id = -1;
        box = {0, 0, 0, 0};
    }
} DetectedObject;

typedef struct DetectedObjectsGroup {
    int id;
    int count;
    // std::vector<DetectedObject> results(OBJ_NUM_MAX_SIZE);
    DetectedObject results[OBJ_NUM_MAX_SIZE];
    float once_npu_run;
    int focused_box_id;
    int target_box_id;
    float cur_framerate;
    DetectedObjectsGroup(): id(0), count(0), once_npu_run(0.0f), focused_box_id(-1), target_box_id(-1), cur_framerate(0.0f) {}
    ~DetectedObjectsGroup() {}
    void Dump() {
        // LogInfo("DetectedObjectsGroup: id=%d, count=%d\n", id, count);
        std::cout << "DetectedObjectsGroup: id=" << id << ", count=" << count << std::endl;
        for (int i = 0; i < count; i++) {
            std::cout << "  " << results[i] << std::endl;
        }
    }
    void Reset() {
        count = 0;
        once_npu_run = 0.0f;
        focused_box_id = -1;
        target_box_id = -1;
        cur_framerate = 0.0f;
        for (auto& result : results) {
            result.Reset();
        }
    }
} DetectedObjectsGroup;

typedef enum { YOLOV5 = 0, YOLOV7, MODEL_TYPE_UNDEFINED } ModelType;

} // namespace vision