# spec2.md — proc_realsense (B) & proc_npu (C) Architecture Specification

## 1. System Context

Current system contains a main process **A** implemented in C that is responsible for:

* System orchestration
* Command routing
* Watchdog supervision
* Log aggregation

Two new processes are introduced:

| Process | Name           | Responsibility                                |
| ------- | -------------- | --------------------------------------------- |
| B       | proc_realsense | RealSense D435 acquisition + frame publishing |
| C       | proc_npu       | NPU inference + result publishing             |

Design principles:

* **Control plane** → Unix Domain Socket (UDS)
* **Data plane** → Shared Memory Ring Buffer (zero‑copy)
* **Process isolation** → failures in B/C must not crash A

---

# 2. Repository Layout

Existing project structure is preserved and extended:

```
UAV_ROBOT/

  uav_robotd/
    app/
    core/
    drv/

  common/
    include/
      abi/
        msg_types.h
        shm_ring.h
        ipc_framing.h

  proc_realsense/
    Makefile
    src/
      main.cpp
      rs_capture.cpp
      rs_capture.h
      shm_writer.cpp
      shm_writer.h
      ctrl_server.cpp
      ctrl_server.h
      health.cpp
      health.h

  proc_npu/
    Makefile
    src/
      main.cpp
      shm_reader.cpp
      shm_reader.h
      preprocess.cpp
      preprocess.h
      npu_infer.cpp
      npu_infer.h
      postprocess.cpp
      postprocess.h
      ctrl_server.cpp
      ctrl_server.h
      health.cpp
      health.h

  Makefile
  spec2.md
```

All processes include common ABI headers from:

```
common/include/abi/
```

---

# 3. IPC Architecture

## 3.1 Data Plane (High Bandwidth)

Used for image data transfer.

Mechanism:

```
Shared Memory Ring Buffer
```

Flow:

```
RealSense (B)
    ↓
Shared Memory Ring
    ↓
NPU Process (C)
```

Properties:

* zero‑copy between processes
* fixed slot size
* lock‑free state machine

### Slot State Machine

```
FREE → WRITING → READY → READING → FREE
```

Producer (B):

1. find FREE slot
2. mark WRITING
3. write frame data
4. mark READY
5. notify consumer

Consumer (C):

1. select READY slot
2. mark READING
3. run inference
4. mark FREE

### Frame Notification

Recommended mechanism:

```
eventfd + epoll
```

Alternative:

```
UDS datagram: FRAME_READY(slot_id, frame_id)
```

---

# 4. Shared Memory Ring Definition

Each slot contains:

```
struct FrameSlot
{
    uint32_t state;

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

    uint32_t color_offset;
    uint32_t depth_offset;

    uint32_t payload_size;
};
```

Payload layout:

```
[color image]
[depth image]
```

### Default Frame Format

Color:

```
BGR8
```

Depth:

```
Z16
```

Aligned depth:

```
Depth aligned to color frame
```

### Default Configuration

```
Resolution : 640x480
FPS        : 30
Ring slots : 8
```

---

# 5. Process B — proc_realsense

## 5.1 Responsibilities

Process B handles:

* RealSense pipeline initialization
* Frame capture
* Depth alignment
* Optional filtering
* Writing frames to shared memory
* System health reporting

## 5.2 Input Commands (from A)

| Command        | Description              |
| -------------- | ------------------------ |
| B_START        | start capture            |
| B_STOP         | stop capture             |
| B_SET_PROFILE  | configure resolution/FPS |
| B_SET_EXPOSURE | exposure control         |
| B_SET_ROI      | region of interest       |
| B_SNAPSHOT     | capture diagnostic frame |

## 5.3 Output Messages

Heartbeat:

```
B_HEARTBEAT
{
  timestamp
  fps
  drop_count
}
```

Status:

```
B_STATUS
{
  state
  error_code
}
```

## 5.4 Internal Modules

### rs_capture

Handles RealSense SDK pipeline

Responsibilities:

* configure streams
* capture frameset
* timestamp handling

### shm_writer

Responsibilities:

* locate free slot
* write frame metadata
* copy payload
* signal consumer

### ctrl_server

Responsibilities:

* receive commands via UDS
* update runtime configuration

### health

Responsibilities:

* FPS statistics
* error counters
* heartbeat generation

## 5.5 Thread Model

Recommended threads:

```
T_capture
T_publish
T_ctrl
```

Capture thread:

```
wait_for_frames()
```

Publish thread:

```
align + shm write
```

## 5.6 Failure Handling

Possible errors:

* camera disconnected
* frame timeout
* pipeline failure

Recovery strategy:

1. retry pipeline restart
2. report B_STATUS
3. supervisor (A) may restart process

---

# 6. Process C — proc_npu

## 6.1 Responsibilities

Process C performs:

* frame ingestion
* preprocessing
* NPU inference
* post‑processing
* result reporting

## 6.2 Input Commands (from A)

| Command         | Description                |
| --------------- | -------------------------- |
| C_START         | enable inference           |
| C_STOP          | disable inference          |
| C_LOAD_MODEL    | load NPU model             |
| C_UNLOAD_MODEL  | unload model               |
| C_SET_THRESHOLD | adjust detection threshold |
| C_SET_RATE      | limit inference FPS        |

## 6.3 Output Messages

Heartbeat:

```
C_HEARTBEAT
{
  timestamp
  inference_fps
  avg_latency_ms
}
```

Result message:

```
C_RESULT
{
  frame_id
  num_detections

  detections[]
  {
    class_id
    score
    x1
    y1
    x2
    y2

    xyz_mm (optional)
  }
}
```

## 6.4 Internal Modules

### shm_reader

Responsibilities:

* wait for frame notification
* fetch READY slot
* release slot

### preprocess

Responsibilities:

* resize
* normalization
* tensor preparation

### npu_infer

Responsibilities:

* load model
* execute inference

### postprocess

Responsibilities:

* decode network output
* NMS
* optional depth fusion

### health

Responsibilities:

* latency statistics
* inference rate

## 6.5 Thread Model

Recommended threads:

```
T_ingest
T_infer
```

Strategy:

```
latest‑wins
```

Older frames are dropped if inference lags.

## 6.6 Failure Handling

Errors:

* model load failure
* NPU runtime error

Strategy:

1. report C_STATUS
2. stop inference
3. wait for restart command

---

# 7. Makefile Integration

Top level targets:

```
make all
make proc_realsense
make proc_npu
make clean
```

Compilation performed by:

```
$(MAKE) -C proc_realsense
$(MAKE) -C proc_npu
```

Dependencies:

RealSense process:

```
-lrealsense2
-lpthread
```

NPU process:

```
-lrknnrt
-lpthread
```

---

# 8. Default Runtime Configuration

```
Camera resolution : 640x480
Camera FPS        : 30
Ring slots        : 8
Inference mode    : latest-wins
Output            : bbox + optional xyz
```

---

# 9. Acceptance Criteria

## B Process

* stable capture ≥ 60 seconds
* SHM ring slot state transitions valid
* heartbeat every second

## C Process

* model load/unload works
* inference produces valid results
* no backlog accumulation

## System

* A can restart B/C
* failure isolation between processes
* camera disconnect recovery

---

# 10. Future Extensions

Possible upgrades:

* GPU preprocessing
* multi‑camera support
* multiple NPU pipelines
* distributed inference

---

END OF SPEC
