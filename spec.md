# UAV_ROBOT A Process Architecture Spec

## 1. Purpose

The **A process (control / supervisor)** is the central runtime of the system. It is responsible for:

* Device control (robot arm, chassis, gripper, platform lock, relays)
* Event‑driven runtime (reactor / router / bus)
* Communication channels (CAN / UART / mesh / IPC)
* Logging system (system black‑box)
* Managing child processes:

  * **B = visiond** (camera acquisition)
  * **C = inferd** (NPU inference)

Design goals:

* Deterministic control loop
* Event‑driven architecture
* Decoupled modules
* Crash isolation (vision / inference separated)
* Scalable communication channels

---

# 2. Directory Structure

```
app/
  main.c
  tasks/

core/
  msg/
    msg_types.h

  reactor/
    reactor.c
    reactor.h

  channel/
    can_channel.c
    mesh_channel.c
    bc_channel.c
    uart_channel.c

  proto/
    proto_zdt_arm.c
    proto_robomodule_drv.c
    proto_gripper_uart.c
    proto_platform_lock.c
    proto_mesh_link.c

  bus/
    bus.c

  router/
    router.c

  scheduler/
    scheduler.c

  dev/
    arm.c
    car.c
    platform.c
    relay.c
    gripper.c

  log/
    log.c

  supervisor/
    supervisor.c


drv/
  dev/
    can_socketcan.c
    uart_posix.c
    mesh_eth.c

  io/
    gpio_sysfs.c

include/

spec.md
```

---

# 3. Architecture Layers

System architecture is organized into layered modules.

```
app
 ↓
router
 ↓
bus
 ↓
proto
 ↓
dev
 ↓
drv
 ↓
hardware
```

Event entry is handled by:

```
reactor → channel → proto → bus/router
```

---

# 4. Module Responsibilities

## 4.1 reactor

Event loop based on **epoll + timers**.

Responsibilities:

* manage file descriptor events
* manage timers
* trigger channel callbacks

Example:

```
reactor_add_fd(can_fd, can_channel_on_readable)
reactor_add_fd(mesh_fd, mesh_channel_on_readable)
```

---

## 4.2 channel

Channel is the **communication adapter layer**.

It bridges external IO into the system runtime.

Responsibilities:

* read IO streams
* handle reconnect / buffering
* call protocol decoder
* publish events or commands

Example channels:

```
can_channel
mesh_channel
bc_channel
uart_channel
```

Typical flow:

```
IO → channel → proto → Event/Command
```

---

## 4.3 proto (device‑independent protocol codec)

Protocol codec layer.

Responsibilities:

* encode protocol packets
* decode protocol packets
* CRC / framing

Restrictions:

* NO socket
* NO read/write
* NO reactor

Example modules:

```
proto_zdt_arm
proto_robomodule_drv
proto_gripper_uart
proto_platform_lock
proto_mesh_link
```

---

## 4.4 bus (event pub/sub)

Internal event bus.

Responsibilities:

* event broadcasting
* subscriber management

Example:

```
bus_publish(EVENT_DETECT_RESULT)
bus_publish(EVENT_ARM_FEEDBACK)
```

Subscribers may include:

* tasks
* logger
* telemetry
* router

---

## 4.5 router (command dispatch)

Router processes **Command messages**.

Responsibilities:

* route command to target handler

Example:

```
Command.target = CAR
Command.action = SET_VELOCITY

router_dispatch(Command)
```

Router invokes:

```
car_handler()
```

Router is **directed dispatch**, unlike bus broadcast.

---

## 4.6 dev (device abstraction layer)

Device control API.

Responsibilities:

* expose device capability
* hide protocol details

Examples:

```
arm_move()
car_set_velocity()
platform_lock()
gripper_open()
relay_on()
```

dev internally calls:

```
proto encode → drv send
```

Restrictions:

* dev must NOT manage file descriptors
* dev must NOT call reactor

---

## 4.7 drv (hardware IO layer)

Lowest software layer.

Responsibilities:

* device IO
* system calls

Examples:

```
can_send()
uart_write()
socket_read()
```

drv must not know business semantics.

---

## 4.8 log (logging system)

Central logging system in A process.

Design:

* async logging
* log queue
* log thread writes file

Log content:

* system lifecycle
* device commands
* errors
* child process health

---

## 4.9 supervisor

Child process manager.

Responsibilities:

* launch B (visiond)
* launch C (inferd)
* heartbeat monitoring
* restart policy

Startup sequence:

```
fork + exec visiond
fork + exec inferd
```

Shutdown sequence:

```
stop C
stop B
cleanup resources
```

---

# 5. Call Chains

## 5.1 Local Device Control

```
app/tasks
   ↓
core/dev
   ↓
core/proto
   ↓
drv
   ↓
hardware
```

Example:

```
car_set_velocity()
  → proto_robomodule_drv_encode()
  → can_send()
```

---

## 5.2 Mesh Remote Command Flow

```
mesh socket
   ↓
reactor
   ↓
mesh_channel
   ↓
proto_mesh_link_decode
   ↓
Command
   ↓
router_dispatch
   ↓
device handler
   ↓
core/dev
   ↓
drv send
```

---

## 5.3 Device Feedback Flow

```
CAN frame
   ↓
reactor
   ↓
can_channel
   ↓
proto_decode
   ↓
Event
   ↓
bus_publish
   ↓
subscribers
```

Subscribers may include:

* tasks
* logger
* mesh telemetry

---

## 5.4 Inference Result Flow (C → A)

```
inferd
   ↓ socket
bc_channel
   ↓
decode DetectResult
   ↓
bus_publish
   ↓
task logic
   ↓
core/dev
```

A process retains final control authority.

---

## 5.5 Child Process Lifecycle

Startup:

```
supervisor
   ↓
create IPC resources
   ↓
fork visiond
   ↓
fork inferd
```

Shutdown:

```
stop inferd
stop visiond
cleanup sockets/shm
```

---

# 6. Dependency Rules

Allowed dependencies:

```
app → dev

dev → proto → drv

channel → drv + proto + bus/router

proto → no IO

drv → no business logic
```

---

# 7. Runtime Principles

1. Reactor thread must never block
2. Vision / inference must run in external processes
3. Protocol layer must remain pure codec
4. Device actions must be centralized in dev layer
5. Events should propagate via bus

---

# 8. Summary

The A process acts as:

* control runtime
* device manager
* communication hub
* event system
* logging center
* process supervisor

The architecture ensures:

* modular design
* clear separation of responsibilities
* safe robot control
* scalability for future extensions
