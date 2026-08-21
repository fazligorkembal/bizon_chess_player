# Hardware Bring-Up Implementation Plan (Host Side)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive one physical Bizon arm from the existing ROS 2 stack over a framed serial link to a BlackPill, with a fault chain that actually aborts motion when the hardware disagrees.

**Architecture:** Add a third `ros2_control_hardware_type` branch (`serial`) alongside the existing `rviz` and `isaac` branches, feeding a dumb step-executing MCU. The MCU's reported step counters — never the echoed command — become `state_interfaces`, so JointTrajectoryController tolerance violations propagate up through MoveIt to the tree's RecoveryNode. Chess geometry moves out of the decision node and into the already-tested `bizon_chess` package so it can be calibrated.

**Tech Stack:** ROS 2 Humble, ros2_control, MoveIt2, BehaviorTree.CPP v4, ament_cmake + gtest, POSIX termios, STM32F411 (firmware, out of scope here).

**Spec:** `docs/superpowers/specs/2026-08-21-hardware-bringup-design.md`

## Scope

This plan covers **host-side ROS 2 work only**. MCU firmware is a separate project with a
separate toolchain; Task 1 defines the wire contract it must satisfy, and Task 2 ships a
pty-based loopback stub so every task below is testable with no hardware present.

Tasks 1-3, 6, 7 and 8 need no hardware. Tasks 4, 5 and 9 have a bench step that does.

## Global Constraints

- ROS 2 distro: **Humble**. C++ standard: **17** for `bizon_behavior_clients`, **20** for `bizon_behavior_servers`.
- Single robot for this plan. Namespace `/bizon2`. The `bizon3` params file is not touched.
- Build from `ros_env/`: `colcon build --symlink-install --packages-up-to <pkg>`, then `. install/setup.bash`.
- Test: `colcon test --packages-select <pkg> && colcon test-result --verbose`.
- No ROS 2 on the host machine. Builds run in `docker-bizon_chess_player:latest`, and this shell
  predates the user's `docker` group membership, so every docker call is wrapped `sg docker -c "..."`:

      sg docker -c "docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp \
        -v /home/gorkem/Documents/projects/bizon_chess_player:/ws \
        -v /home/gorkem/.cache/bizon-sdd:/cb -w /ws/ros_env \
        docker-bizon_chess_player:latest bash -c \
        'source /opt/ros/humble/setup.bash && colcon build --packages-select <pkg> \
         --build-base /cb/build --install-base /cb/install'"

- Do not touch `ros_env/src/BehaviorTree.CPP/` or `Stockfish/` — upstream clones.
- **No behavior may change in a task without a test in that same task.**
- `uncrustify` currently fails on pre-existing files (`src/main.cpp`, `test/test_arm_action_client_node.cpp`,
  `test/test_chess_game_tree_structure.cpp`, `test/test_recovery_node.cpp`). Do not treat that as a
  regression; do keep new files clean.

## Dependency on the pre-hardware plan

`docs/superpowers/plans/2026-08-19-pre-hardware-refactor.md` is still mid-flight: Tasks 1-3 are
committed, Task 4's `arm_plugin.cpp` and `Arm.action` exist on disk uncommitted, Tasks 5-7 are
not started. This plan does not duplicate them. Task 6 of that plan (lifecycle manager and the
E-stop path) is a hard prerequisite for physical power-on and is referenced, not re-specified.

## File Structure

```
bizon_ros2_control/
  include/bizon_ros2_control/bizon_protocol.hpp     NEW  wire format, CRC, pack/unpack. No ROS, no I/O.
  include/bizon_ros2_control/serial_comm.hpp        NEW  termios transport + framing/resync
  src/serial_comm.cpp                               NEW
  src/bizon_system_interface.cpp                    MOD  serial branch, converters, homing
  include/bizon_ros2_control/bizon_system_interface.hpp  MOD
  test/test_bizon_protocol.cpp                      NEW
  test/test_serial_comm.cpp                         NEW  pty loopback
  test/test_step_converters.cpp                     NEW
  test/mcu_stub.hpp                                 NEW  scriptable fake MCU over a pty

bizon_description/urdf/bizon_system_interface.xacro MOD  serial params, gripper mimic
bizon_player_bringup/params/bizon2_full_ros2_controllers.yaml  MOD  JTC tolerances
bizon_player_bringup/params/bizon2_calibration.yaml NEW  RobotParams from measurement

bizon_chess/
  include/bizon_chess/board_geometry.hpp            MOD  add squareToJointAngles
  src/board_geometry.cpp                            MOD

bizon_behavior_clients/
  plugins/action/make_decision_client_node.cpp      MOD  delete duplicate geometry, call bizon_chess
  include/.../make_decision_client_node.hpp         MOD  drop hardcoded constants, add params

bizon_behavior_servers/
  include/.../plugins/board_plugin.hpp              MOD  FEN debounce
  plugins/board_plugin.cpp                          MOD
```

---

### Task 1: Wire protocol, packed and unpacked

The contract the firmware must match. Pure C++, no ROS, no I/O, so it is fully unit testable and
can be copied verbatim into the firmware project.

**Files:**
- Create: `ros_env/src/bizon_ros2_control/include/bizon_ros2_control/bizon_protocol.hpp`
- Create: `ros_env/src/bizon_ros2_control/test/test_bizon_protocol.cpp`
- Modify: `ros_env/src/bizon_ros2_control/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing.
- Produces: `bizon_protocol::kAxisCount` (5), `bizon_protocol::CommandFrame`,
  `bizon_protocol::StateFrame`, `bizon_protocol::crc16(const uint8_t*, size_t)`,
  `bizon_protocol::encode(const CommandFrame&, uint8_t out[kCommandFrameSize])`,
  `bizon_protocol::decode(const uint8_t* in, size_t len, StateFrame& out)`.
  Tasks 2, 3, 4 and 5 consume all of these.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_ros2_control/test/test_bizon_protocol.cpp`:

```cpp
#include <gtest/gtest.h>
#include <cstring>
#include "bizon_ros2_control/bizon_protocol.hpp"

using bizon_protocol::CommandFrame;
using bizon_protocol::StateFrame;
using bizon_protocol::kAxisCount;

TEST(BizonProtocol, CommandFrameRoundTripsThroughEncodeDecode)
{
  CommandFrame cmd{};
  cmd.seq = 4242;
  cmd.flags = bizon_protocol::kFlagEnable | bizon_protocol::kFlagHomeRequest;
  cmd.gripper_current = 12;
  for (int i = 0; i < kAxisCount; ++i) {
    cmd.target_steps[i] = -1000 * (i + 1);
  }

  uint8_t buf[bizon_protocol::kCommandFrameSize];
  bizon_protocol::encode(cmd, buf);

  CommandFrame back{};
  ASSERT_TRUE(bizon_protocol::decodeCommand(buf, sizeof(buf), back));
  EXPECT_EQ(back.seq, cmd.seq);
  EXPECT_EQ(back.flags, cmd.flags);
  EXPECT_EQ(back.gripper_current, cmd.gripper_current);
  for (int i = 0; i < kAxisCount; ++i) {
    EXPECT_EQ(back.target_steps[i], cmd.target_steps[i]) << "axis " << i;
  }
}

TEST(BizonProtocol, StateFrameRoundTrips)
{
  StateFrame st{};
  st.seq = 7;
  st.status = bizon_protocol::kStatusEstop;
  st.homed_bits = 0b11111;
  st.stall_bits = 0b00100;
  for (int i = 0; i < kAxisCount; ++i) {
    st.actual_steps[i] = 77 * (i + 1);
  }

  uint8_t buf[bizon_protocol::kStateFrameSize];
  bizon_protocol::encodeState(st, buf);

  StateFrame back{};
  ASSERT_TRUE(bizon_protocol::decode(buf, sizeof(buf), back));
  EXPECT_EQ(back.seq, st.seq);
  EXPECT_EQ(back.status, st.status);
  EXPECT_EQ(back.homed_bits, st.homed_bits);
  EXPECT_EQ(back.stall_bits, st.stall_bits);
  for (int i = 0; i < kAxisCount; ++i) {
    EXPECT_EQ(back.actual_steps[i], st.actual_steps[i]) << "axis " << i;
  }
}

// A corrupted frame must be rejected, not silently accepted with garbage
// positions. Feeding a bad step count into ros2_control is exactly the failure
// this checksum exists to prevent.
TEST(BizonProtocol, SingleBitFlipIsRejected)
{
  StateFrame st{};
  st.seq = 1;
  for (int i = 0; i < kAxisCount; ++i) {
    st.actual_steps[i] = 1234;
  }

  uint8_t buf[bizon_protocol::kStateFrameSize];
  bizon_protocol::encodeState(st, buf);
  buf[4] ^= 0x01;

  StateFrame back{};
  EXPECT_FALSE(bizon_protocol::decode(buf, sizeof(buf), back));
}

TEST(BizonProtocol, WrongSyncByteIsRejected)
{
  StateFrame st{};
  uint8_t buf[bizon_protocol::kStateFrameSize];
  bizon_protocol::encodeState(st, buf);
  buf[0] = 0x00;

  StateFrame back{};
  EXPECT_FALSE(bizon_protocol::decode(buf, sizeof(buf), back));
}

TEST(BizonProtocol, ShortBufferIsRejected)
{
  StateFrame st{};
  uint8_t buf[bizon_protocol::kStateFrameSize];
  bizon_protocol::encodeState(st, buf);

  StateFrame back{};
  EXPECT_FALSE(bizon_protocol::decode(buf, sizeof(buf) - 1, back));
}

// Known-answer test. The firmware implements CRC16/CCITT-FALSE independently;
// if the two ever disagree, every frame is rejected and the arm simply stops.
// Pinning the vector here makes that a compile-time-visible contract.
TEST(BizonProtocol, Crc16MatchesCcittFalseKnownAnswer)
{
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(bizon_protocol::crc16(data, sizeof(data)), 0x29B1);
}
```

- [ ] **Step 2: Run the test and verify it fails**

```bash
sg docker -c "docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp \
  -v /home/gorkem/Documents/projects/bizon_chess_player:/ws \
  -v /home/gorkem/.cache/bizon-sdd:/cb -w /ws/ros_env \
  docker-bizon_chess_player:latest bash -c \
  'source /opt/ros/humble/setup.bash && colcon build --packages-select bizon_ros2_control \
   --build-base /cb/build --install-base /cb/install'"
```

Expected: FAIL — `bizon_ros2_control/bizon_protocol.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`ros_env/src/bizon_ros2_control/include/bizon_ros2_control/bizon_protocol.hpp`:

```cpp
#ifndef BIZON_ROS2_CONTROL__BIZON_PROTOCOL_HPP_
#define BIZON_ROS2_CONTROL__BIZON_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>

/// Wire format between the host and the BlackPill axis controller.
///
/// Deliberately free of ROS and of any I/O so the firmware project can compile
/// this same header. Positions travel as integer steps: no floating-point drift
/// across the link, and the MCU never needs the kinematic model.
namespace bizon_protocol
{

inline constexpr int kAxisCount = 5;          ///< rev1, pris1, rev2, rev3, gripper
inline constexpr uint8_t kSyncByte = 0xA5;

// Command flags, host -> MCU.
inline constexpr uint16_t kFlagEnable = 1u << 0;
inline constexpr uint16_t kFlagHomeRequest = 1u << 1;
inline constexpr uint16_t kFlagClearFault = 1u << 2;

// Status bits, MCU -> host.
inline constexpr uint16_t kStatusEstop = 1u << 0;
inline constexpr uint16_t kStatusFault = 1u << 1;
inline constexpr uint16_t kStatusHoming = 1u << 2;

struct CommandFrame
{
  uint16_t seq = 0;
  uint16_t flags = 0;
  int32_t target_steps[kAxisCount] = {0, 0, 0, 0, 0};
  uint8_t gripper_current = 0;   ///< TMC2209 IRUN scale, 0-31
};

struct StateFrame
{
  uint16_t seq = 0;              ///< echoes the command that produced this state
  uint16_t status = 0;
  int32_t actual_steps[kAxisCount] = {0, 0, 0, 0, 0};
  uint8_t homed_bits = 0;
  uint8_t endstop_bits = 0;
  uint8_t stall_bits = 0;
  uint8_t fault_code = 0;
};

// sync(1) seq(2) flags(2) steps(20) gripper_current(1) crc(2)
inline constexpr size_t kCommandFrameSize = 28;
// sync(1) seq(2) status(2) steps(20) homed(1) endstop(1) stall(1) fault(1) crc(2)
inline constexpr size_t kStateFrameSize = 31;

/// CRC16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
inline uint16_t crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

namespace detail
{
inline void putU16(uint8_t * p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
inline uint16_t getU16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | static_cast<uint16_t>(p[1] << 8);
}
inline void putI32(uint8_t * p, int32_t v)
{
  const uint32_t u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u & 0xFF);
  p[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}
inline int32_t getI32(const uint8_t * p)
{
  const uint32_t u = static_cast<uint32_t>(p[0]) |
    (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) |
    (static_cast<uint32_t>(p[3]) << 24);
  return static_cast<int32_t>(u);
}
}  // namespace detail

inline void encode(const CommandFrame & cmd, uint8_t out[kCommandFrameSize])
{
  out[0] = kSyncByte;
  detail::putU16(out + 1, cmd.seq);
  detail::putU16(out + 3, cmd.flags);
  for (int i = 0; i < kAxisCount; ++i) {
    detail::putI32(out + 5 + 4 * i, cmd.target_steps[i]);
  }
  out[25] = cmd.gripper_current;
  detail::putU16(out + 26, crc16(out, 26));
}

inline bool decodeCommand(const uint8_t * in, size_t len, CommandFrame & out)
{
  if (len != kCommandFrameSize || in[0] != kSyncByte) {
    return false;
  }
  if (crc16(in, 26) != detail::getU16(in + 26)) {
    return false;
  }
  out.seq = detail::getU16(in + 1);
  out.flags = detail::getU16(in + 3);
  for (int i = 0; i < kAxisCount; ++i) {
    out.target_steps[i] = detail::getI32(in + 5 + 4 * i);
  }
  out.gripper_current = in[25];
  return true;
}

inline void encodeState(const StateFrame & st, uint8_t out[kStateFrameSize])
{
  out[0] = kSyncByte;
  detail::putU16(out + 1, st.seq);
  detail::putU16(out + 3, st.status);
  for (int i = 0; i < kAxisCount; ++i) {
    detail::putI32(out + 5 + 4 * i, st.actual_steps[i]);
  }
  out[25] = st.homed_bits;
  out[26] = st.endstop_bits;
  out[27] = st.stall_bits;
  out[28] = st.fault_code;
  detail::putU16(out + 29, crc16(out, 29));
}

inline bool decode(const uint8_t * in, size_t len, StateFrame & out)
{
  if (len != kStateFrameSize || in[0] != kSyncByte) {
    return false;
  }
  if (crc16(in, 29) != detail::getU16(in + 29)) {
    return false;
  }
  out.seq = detail::getU16(in + 1);
  out.status = detail::getU16(in + 3);
  for (int i = 0; i < kAxisCount; ++i) {
    out.actual_steps[i] = detail::getI32(in + 5 + 4 * i);
  }
  out.homed_bits = in[25];
  out.endstop_bits = in[26];
  out.stall_bits = in[27];
  out.fault_code = in[28];
  return true;
}

}  // namespace bizon_protocol

#endif  // BIZON_ROS2_CONTROL__BIZON_PROTOCOL_HPP_
```

- [ ] **Step 4: Register the test in CMakeLists.txt**

Append inside the existing `if(BUILD_TESTING)` block of
`ros_env/src/bizon_ros2_control/CMakeLists.txt` (create the block if the package has none):

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bizon_protocol test/test_bizon_protocol.cpp)
  target_include_directories(test_bizon_protocol PRIVATE include)
endif()
```

- [ ] **Step 5: Run the test and verify it passes**

```bash
sg docker -c "docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp \
  -v /home/gorkem/Documents/projects/bizon_chess_player:/ws \
  -v /home/gorkem/.cache/bizon-sdd:/cb -w /ws/ros_env \
  docker-bizon_chess_player:latest bash -c \
  'source /opt/ros/humble/setup.bash && \
   colcon build --packages-select bizon_ros2_control --build-base /cb/build --install-base /cb/install && \
   colcon test --packages-select bizon_ros2_control --build-base /cb/build --install-base /cb/install && \
   colcon test-result --test-result-base /cb/build --verbose'"
```

Expected: 6 tests, all PASS.

- [ ] **Step 6: Commit**

```bash
git add ros_env/src/bizon_ros2_control/include/bizon_ros2_control/bizon_protocol.hpp \
        ros_env/src/bizon_ros2_control/test/test_bizon_protocol.cpp \
        ros_env/src/bizon_ros2_control/CMakeLists.txt
git commit -m "feat(hw): define the host/MCU wire protocol with a checksummed frame codec"
```

---

### Task 2: Serial transport and the pty-based MCU stub

Fills the `// todo: add serialComm.h` slot that has been sitting in
`bizon_system_interface.hpp` since the package was written. The stub is not throwaway — it is
how Tasks 3, 4 and 5 get tested without hardware, and how regressions get caught after the
hardware exists.

**Files:**
- Create: `ros_env/src/bizon_ros2_control/include/bizon_ros2_control/serial_comm.hpp`
- Create: `ros_env/src/bizon_ros2_control/src/serial_comm.cpp`
- Create: `ros_env/src/bizon_ros2_control/test/mcu_stub.hpp`
- Create: `ros_env/src/bizon_ros2_control/test/test_serial_comm.cpp`
- Modify: `ros_env/src/bizon_ros2_control/CMakeLists.txt`

**Interfaces:**
- Consumes: everything `bizon_protocol` produces in Task 1.
- Produces: `bizon_ros2_control::SerialComm` with
  `bool open(const std::string & device, int baudrate)`,
  `void close()`,
  `bool isOpen() const`,
  `bool send(const bizon_protocol::CommandFrame &)`,
  `bool receive(bizon_protocol::StateFrame & out)` (non-blocking; false when no complete valid
  frame is buffered),
  `size_t framesDropped() const`.
  Task 3 consumes all of these. `McuStub` (test-only) produces
  `std::string devicePath()`, `void setActualSteps(int axis, int32_t)`, `void setStatus(uint16_t)`,
  `void setHomedBits(uint8_t)`, `void setEchoTargets(bool)`, `void pump()`.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_ros2_control/test/test_serial_comm.cpp`:

```cpp
#include <gtest/gtest.h>
#include "bizon_ros2_control/serial_comm.hpp"
#include "mcu_stub.hpp"

using bizon_ros2_control::SerialComm;
using bizon_ros2_control::testing::McuStub;

TEST(SerialComm, OpensThePtyAndReportsOpen)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));
  EXPECT_TRUE(comm.isOpen());
  comm.close();
  EXPECT_FALSE(comm.isOpen());
}

TEST(SerialComm, OpeningAMissingDeviceFailsWithoutThrowing)
{
  SerialComm comm;
  EXPECT_FALSE(comm.open("/dev/definitely-not-a-tty", 115200));
  EXPECT_FALSE(comm.isOpen());
}

TEST(SerialComm, SendReachesTheStubAndStateComesBack)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  stub.setActualSteps(0, 1234);
  stub.setActualSteps(3, -99);
  stub.setHomedBits(0b11111);

  bizon_protocol::CommandFrame cmd{};
  cmd.seq = 5;
  cmd.flags = bizon_protocol::kFlagEnable;
  ASSERT_TRUE(comm.send(cmd));

  stub.pump();

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.seq, 5);
  EXPECT_EQ(st.actual_steps[0], 1234);
  EXPECT_EQ(st.actual_steps[3], -99);
  EXPECT_EQ(st.homed_bits, 0b11111);
}

TEST(SerialComm, ReceiveReturnsFalseWhenNothingIsBuffered)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  bizon_protocol::StateFrame st{};
  EXPECT_FALSE(comm.receive(st));
}

// A byte lost on the wire must cost one frame, not the link. Without resync the
// reader stays permanently misaligned and every subsequent frame fails CRC,
// which on real hardware looks exactly like a dead MCU.
TEST(SerialComm, ResyncsAfterGarbagePrecedesAValidFrame)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  stub.writeRaw({0x00, 0x11, 0x22});
  stub.setActualSteps(1, 4242);
  stub.emitState(9);

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.seq, 9);
  EXPECT_EQ(st.actual_steps[1], 4242);
  EXPECT_GE(comm.framesDropped(), 1u);
}

TEST(SerialComm, HandlesAFrameSplitAcrossTwoReads)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  stub.setActualSteps(2, 777);
  stub.emitStateSplit(11, 7);   // first 7 bytes, then the rest

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.seq, 11);
  EXPECT_EQ(st.actual_steps[2], 777);
}
```

- [ ] **Step 2: Run the test and verify it fails**

Same docker build command as Task 1 Step 2.
Expected: FAIL — `bizon_ros2_control/serial_comm.hpp: No such file or directory`.

- [ ] **Step 3: Write the transport header**

`ros_env/src/bizon_ros2_control/include/bizon_ros2_control/serial_comm.hpp`:

```cpp
#ifndef BIZON_ROS2_CONTROL__SERIAL_COMM_HPP_
#define BIZON_ROS2_CONTROL__SERIAL_COMM_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "bizon_ros2_control/bizon_protocol.hpp"

namespace bizon_ros2_control
{

/// Non-blocking framed transport to the MCU over a tty.
///
/// receive() is deliberately non-blocking and returns false rather than waiting:
/// it is called from ros2_control's read(), which runs in the controller
/// manager's update loop and must never stall on I/O.
class SerialComm
{
public:
  SerialComm() = default;
  ~SerialComm();

  SerialComm(const SerialComm &) = delete;
  SerialComm & operator=(const SerialComm &) = delete;

  bool open(const std::string & device, int baudrate);
  void close();
  bool isOpen() const {return fd_ >= 0;}

  bool send(const bizon_protocol::CommandFrame & cmd);

  /// Drains whatever is readable, then returns the newest complete valid frame.
  /// False when none is available.
  bool receive(bizon_protocol::StateFrame & out);

  /// Frames discarded for bad sync or bad CRC since open(). A steadily rising
  /// count means a wiring or baud-rate problem, not a software one.
  size_t framesDropped() const {return frames_dropped_;}

private:
  int fd_ = -1;
  std::vector<uint8_t> rx_;
  size_t frames_dropped_ = 0;
};

}  // namespace bizon_ros2_control

#endif  // BIZON_ROS2_CONTROL__SERIAL_COMM_HPP_
```

- [ ] **Step 4: Write the transport implementation**

`ros_env/src/bizon_ros2_control/src/serial_comm.cpp`:

```cpp
#include "bizon_ros2_control/serial_comm.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>

namespace bizon_ros2_control
{
namespace
{
speed_t toSpeed(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B0;
  }
}
}  // namespace

SerialComm::~SerialComm()
{
  close();
}

bool SerialComm::open(const std::string & device, int baudrate)
{
  close();

  const speed_t speed = toSpeed(baudrate);
  if (speed == B0) {
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }

  ::cfmakeraw(&tty);
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }

  ::tcflush(fd_, TCIOFLUSH);
  rx_.clear();
  frames_dropped_ = 0;
  return true;
}

void SerialComm::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rx_.clear();
}

bool SerialComm::send(const bizon_protocol::CommandFrame & cmd)
{
  if (fd_ < 0) {
    return false;
  }
  uint8_t buf[bizon_protocol::kCommandFrameSize];
  bizon_protocol::encode(cmd, buf);

  size_t written = 0;
  while (written < sizeof(buf)) {
    const ssize_t n = ::write(fd_, buf + written, sizeof(buf) - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

bool SerialComm::receive(bizon_protocol::StateFrame & out)
{
  if (fd_ < 0) {
    return false;
  }

  uint8_t chunk[256];
  for (;;) {
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n > 0) {
      rx_.insert(rx_.end(), chunk, chunk + n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    break;
  }

  // Keep the newest good frame. Under normal operation exactly one arrives per
  // cycle; if the loop ever falls behind, the freshest position is the useful
  // one, not the oldest.
  bool found = false;
  size_t consumed_to = 0;
  size_t i = 0;
  while (i + bizon_protocol::kStateFrameSize <= rx_.size()) {
    if (rx_[i] != bizon_protocol::kSyncByte) {
      ++i;
      continue;
    }
    bizon_protocol::StateFrame candidate{};
    if (bizon_protocol::decode(&rx_[i], bizon_protocol::kStateFrameSize, candidate)) {
      out = candidate;
      found = true;
      i += bizon_protocol::kStateFrameSize;
      consumed_to = i;
    } else {
      ++frames_dropped_;
      ++i;
    }
  }

  if (found) {
    rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(consumed_to));
  } else if (rx_.size() > 4 * bizon_protocol::kStateFrameSize) {
    // Nothing decodable and the buffer is growing: drop all but a frame's worth
    // so a permanently broken link cannot grow unbounded.
    rx_.erase(rx_.begin(), rx_.end() - static_cast<long>(bizon_protocol::kStateFrameSize));
  }
  return found;
}

}  // namespace bizon_ros2_control
```

- [ ] **Step 5: Write the MCU stub**

`ros_env/src/bizon_ros2_control/test/mcu_stub.hpp`:

```cpp
#ifndef BIZON_ROS2_CONTROL__TEST__MCU_STUB_HPP_
#define BIZON_ROS2_CONTROL__TEST__MCU_STUB_HPP_

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

#include "bizon_ros2_control/bizon_protocol.hpp"

namespace bizon_ros2_control
{
namespace testing
{

/// A scriptable fake MCU on the far end of a pty pair.
///
/// This is not throwaway scaffolding: the whole fault chain -- stalled counter,
/// endstop, E-stop -- is only testable in CI because this exists. Keep it after
/// the real hardware arrives.
class McuStub
{
public:
  McuStub()
  {
    char name[256];
    if (::openpty(&master_fd_, &slave_fd_, name, nullptr, nullptr) != 0) {
      master_fd_ = slave_fd_ = -1;
      return;
    }
    device_path_ = name;
    ::fcntl(master_fd_, F_SETFL, O_NONBLOCK);
  }

  ~McuStub()
  {
    if (master_fd_ >= 0) {::close(master_fd_);}
    if (slave_fd_ >= 0) {::close(slave_fd_);}
  }

  McuStub(const McuStub &) = delete;
  McuStub & operator=(const McuStub &) = delete;

  std::string devicePath() const {return device_path_;}

  void setActualSteps(int axis, int32_t steps) {state_.actual_steps[axis] = steps;}
  void setStatus(uint16_t status) {state_.status = status;}
  void setHomedBits(uint8_t bits) {state_.homed_bits = bits;}
  void setStallBits(uint8_t bits) {state_.stall_bits = bits;}
  void setEndstopBits(uint8_t bits) {state_.endstop_bits = bits;}

  /// When true the stub behaves like a perfect actuator, echoing every target
  /// straight back as the measured position. Use it ONLY to prove that the
  /// tolerance chain does *not* fire; never as the default.
  void setEchoTargets(bool echo) {echo_targets_ = echo;}

  /// Reads one command if present and replies with the current state.
  void pump()
  {
    uint8_t buf[bizon_protocol::kCommandFrameSize];
    const ssize_t n = ::read(master_fd_, buf, sizeof(buf));
    uint16_t seq = state_.seq;
    if (n == static_cast<ssize_t>(sizeof(buf))) {
      bizon_protocol::CommandFrame cmd{};
      if (bizon_protocol::decodeCommand(buf, sizeof(buf), cmd)) {
        seq = cmd.seq;
        last_command_ = cmd;
        if (echo_targets_) {
          for (int i = 0; i < static_cast<size_t>(bizon_protocol::kAxisCount); ++i) {
            state_.actual_steps[i] = cmd.target_steps[i];
          }
        }
        if (cmd.flags & bizon_protocol::kFlagHomeRequest) {
          state_.homed_bits = 0b11111;
        }
      }
    }
    emitState(seq);
  }

  void emitState(uint16_t seq)
  {
    state_.seq = seq;
    uint8_t buf[bizon_protocol::kStateFrameSize];
    bizon_protocol::encodeState(state_, buf);
    ssize_t ignored = ::write(master_fd_, buf, sizeof(buf));
    (void)ignored;
  }

  /// Emits a frame in two writes, to exercise the reader's reassembly.
  void emitStateSplit(uint16_t seq, size_t first_chunk)
  {
    state_.seq = seq;
    uint8_t buf[bizon_protocol::kStateFrameSize];
    bizon_protocol::encodeState(state_, buf);
    ssize_t a = ::write(master_fd_, buf, first_chunk);
    ssize_t b = ::write(master_fd_, buf + first_chunk, sizeof(buf) - first_chunk);
    (void)a; (void)b;
  }

  void writeRaw(std::initializer_list<uint8_t> bytes)
  {
    std::vector<uint8_t> v(bytes);
    ssize_t ignored = ::write(master_fd_, v.data(), v.size());
    (void)ignored;
  }

  const bizon_protocol::CommandFrame & lastCommand() const {return last_command_;}

private:
  int master_fd_ = -1;
  int slave_fd_ = -1;
  std::string device_path_;
  bool echo_targets_ = false;
  bizon_protocol::StateFrame state_{};
  bizon_protocol::CommandFrame last_command_{};
};

}  // namespace testing
}  // namespace bizon_ros2_control

#endif  // BIZON_ROS2_CONTROL__TEST__MCU_STUB_HPP_
```

- [ ] **Step 6: Wire the build**

In `ros_env/src/bizon_ros2_control/CMakeLists.txt`, add `src/serial_comm.cpp` to the package
library's sources, and inside `if(BUILD_TESTING)`:

```cmake
  ament_add_gtest(test_serial_comm test/test_serial_comm.cpp src/serial_comm.cpp)
  target_include_directories(test_serial_comm PRIVATE include test)
  target_link_libraries(test_serial_comm util)   # openpty
```

- [ ] **Step 7: Run the tests and verify they pass**

Same docker test command as Task 1 Step 5.
Expected: the 6 protocol tests plus 6 serial tests, all PASS.

- [ ] **Step 8: Commit**

```bash
git add ros_env/src/bizon_ros2_control/include/bizon_ros2_control/serial_comm.hpp \
        ros_env/src/bizon_ros2_control/src/serial_comm.cpp \
        ros_env/src/bizon_ros2_control/test/mcu_stub.hpp \
        ros_env/src/bizon_ros2_control/test/test_serial_comm.cpp \
        ros_env/src/bizon_ros2_control/CMakeLists.txt
git commit -m "feat(hw): add framed serial transport and a scriptable pty MCU stub"
```

---

### Task 3: Step converters and the `serial` hardware branch

Implements the two converters that have been declared in
`bizon_system_interface.hpp` without a definition since the package was written, and fills in
the third arm of the `ros2_control_hardware_type` dispatch that already handles `rviz` and
`isaac`.

**Files:**
- Create: `ros_env/src/bizon_ros2_control/test/test_step_converters.cpp`
- Modify: `ros_env/src/bizon_ros2_control/include/bizon_ros2_control/bizon_system_interface.hpp`
- Modify: `ros_env/src/bizon_ros2_control/src/bizon_system_interface.cpp`
- Modify: `ros_env/src/bizon_description/urdf/bizon_system_interface.xacro`
- Modify: `ros_env/src/bizon_ros2_control/CMakeLists.txt`

**Interfaces:**
- Consumes: `SerialComm`, `bizon_protocol::CommandFrame`, `bizon_protocol::StateFrame`, `McuStub`.
- Produces: a working `ros2_control_hardware_type=serial` branch. Per-joint xacro parameters
  `steps_per_rev`, `micro_steps`, `gear_ratio`, `screw_lead_m` (prismatic only), and hardware
  parameters `serial_port`, `baudrate`. Tasks 4 and 5 extend this branch.

- [ ] **Step 1: Write the failing converter test**

`ros_env/src/bizon_ros2_control/test/test_step_converters.cpp`:

```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "bizon_ros2_control/step_math.hpp"

using bizon_ros2_control::radiansToSteps;
using bizon_ros2_control::stepsToRadians;
using bizon_ros2_control::metresToSteps;
using bizon_ros2_control::stepsToMetres;

// 1.8 deg motor, 1/16 microstepping, direct drive: 200 * 16 = 3200 steps/rev.
TEST(StepMath, FullRevolutionIsMicrostepsTimesFullSteps)
{
  EXPECT_EQ(radiansToSteps(2.0 * M_PI, 16, 1.8, 1.0), 3200);
}

TEST(StepMath, HalfRevolutionIsHalfTheSteps)
{
  EXPECT_EQ(radiansToSteps(M_PI, 16, 1.8, 1.0), 1600);
}

TEST(StepMath, NegativeAnglesGiveNegativeSteps)
{
  EXPECT_EQ(radiansToSteps(-M_PI, 16, 1.8, 1.0), -1600);
}

TEST(StepMath, GearRatioMultipliesTheStepCount)
{
  EXPECT_EQ(radiansToSteps(2.0 * M_PI, 16, 1.8, 5.0), 16000);
}

TEST(StepMath, RadiansRoundTripWithinOneStep)
{
  const double angle = 1.234;
  const int steps = radiansToSteps(angle, 16, 1.8, 1.0);
  EXPECT_NEAR(stepsToRadians(steps, 16, 1.8, 1.0), angle, 2.0 * M_PI / 3200.0);
}

// T8 leadscrew, 2 mm lead: one motor revolution advances 2 mm.
TEST(StepMath, OneLeadOfTravelIsOneMotorRevolution)
{
  EXPECT_EQ(metresToSteps(0.002, 16, 1.8, 1.0, 0.002), 3200);
}

TEST(StepMath, PrismaticTravelRoundTripsWithinOneStep)
{
  const double travel = 0.155;
  const int steps = metresToSteps(travel, 16, 1.8, 1.0, 0.002);
  EXPECT_NEAR(stepsToMetres(steps, 16, 1.8, 1.0, 0.002), travel, 0.002 / 3200.0);
}

// A zero or negative lead would divide by zero and hand the driver a garbage
// step count. Reject it at the boundary instead.
TEST(StepMath, ZeroLeadYieldsZeroRatherThanInfinity)
{
  EXPECT_EQ(metresToSteps(0.1, 16, 1.8, 1.0, 0.0), 0);
}

TEST(StepMath, ZeroStepAngleYieldsZeroRatherThanInfinity)
{
  EXPECT_EQ(radiansToSteps(1.0, 16, 0.0, 1.0), 0);
}
```

- [ ] **Step 2: Run it and verify it fails**

Expected: FAIL — `bizon_ros2_control/step_math.hpp: No such file or directory`.

- [ ] **Step 3: Write `step_math.hpp`**

`ros_env/src/bizon_ros2_control/include/bizon_ros2_control/step_math.hpp`:

```cpp
#ifndef BIZON_ROS2_CONTROL__STEP_MATH_HPP_
#define BIZON_ROS2_CONTROL__STEP_MATH_HPP_

#include <cmath>

namespace bizon_ros2_control
{

/// Microsteps per motor revolution, e.g. 200 * 16 = 3200 for a 1.8 deg motor.
inline double microstepsPerRev(int micro_steps, double step_angle_deg)
{
  if (step_angle_deg <= 0.0 || micro_steps <= 0) {
    return 0.0;
  }
  return (360.0 / step_angle_deg) * static_cast<double>(micro_steps);
}

inline int radiansToSteps(double angle_rad, int micro_steps, double step_angle_deg, double gear_ratio)
{
  const double per_rev = microstepsPerRev(micro_steps, step_angle_deg);
  if (per_rev == 0.0) {
    return 0;
  }
  return static_cast<int>(std::llround(angle_rad / (2.0 * M_PI) * per_rev * gear_ratio));
}

inline double stepsToRadians(int steps, int micro_steps, double step_angle_deg, double gear_ratio)
{
  const double per_rev = microstepsPerRev(micro_steps, step_angle_deg);
  if (per_rev == 0.0 || gear_ratio == 0.0) {
    return 0.0;
  }
  return static_cast<double>(steps) / (per_rev * gear_ratio) * 2.0 * M_PI;
}

/// `screw_lead_m` is the linear travel per motor revolution.
inline int metresToSteps(
  double distance_m, int micro_steps, double step_angle_deg, double gear_ratio, double screw_lead_m)
{
  if (screw_lead_m <= 0.0) {
    return 0;
  }
  const double per_rev = microstepsPerRev(micro_steps, step_angle_deg);
  if (per_rev == 0.0) {
    return 0;
  }
  return static_cast<int>(std::llround(distance_m / screw_lead_m * per_rev * gear_ratio));
}

inline double stepsToMetres(
  int steps, int micro_steps, double step_angle_deg, double gear_ratio, double screw_lead_m)
{
  const double per_rev = microstepsPerRev(micro_steps, step_angle_deg);
  if (per_rev == 0.0 || gear_ratio == 0.0) {
    return 0.0;
  }
  return static_cast<double>(steps) / (per_rev * gear_ratio) * screw_lead_m;
}

}  // namespace bizon_ros2_control

#endif  // BIZON_ROS2_CONTROL__STEP_MATH_HPP_
```

- [ ] **Step 4: Replace the two undefined converter declarations**

In `bizon_system_interface.hpp`, delete these two lines:

```cpp
        int radian_angle_to_step_converter(double angle_rad, int micro_steps, double step_angles_deg, double gear_ratio);
        int distance_to_step_converter(double distance_m, int micro_steps, double step_angles_deg, double gear_ratio);
```

They were never defined, so nothing links against them. `step_math.hpp` replaces both, and being
header-only and ROS-free it is unit testable — which the member functions were not.

Also replace `// todo: add serialComm.h` with `#include "bizon_ros2_control/serial_comm.hpp"`,
replace `// todo: add serialComm uart_comm;` with the members below, and add the serial
read/write declarations next to their `rviz` and `isaac` siblings:

```cpp
        void read_serial_sensors();
        void write_serial_commands();

        struct AxisConfig
        {
          int micro_steps = 16;
          double step_angle_deg = 1.8;
          double gear_ratio = 1.0;
          double screw_lead_m = 0.0;   ///< non-zero marks a prismatic axis
        };

        SerialComm serial_;
        std::vector<AxisConfig> axis_config_;
        bizon_protocol::StateFrame last_state_{};
        uint16_t tx_seq_ = 0;
        bool serial_link_healthy_ = false;
```

- [ ] **Step 5: Add the `serial` branch to `on_init`**

In `bizon_system_interface.cpp`, alongside the existing `rviz` and `isaac` cases:

```cpp
        else if (info_.hardware_parameters.at("ros2_control_hardware_type") == "serial")
        {
            write_commands_ = &BizonSystemInterface::write_serial_commands;
            read_sensors_ = &BizonSystemInterface::read_serial_sensors;

            axis_config_.resize(info.joints.size());
            for (size_t i = 0; i < info.joints.size(); ++i)
            {
                const auto & p = info.joints[i].parameters;
                auto get = [&p](const std::string & key, double fallback) {
                    const auto it = p.find(key);
                    return it == p.end() ? fallback : std::stod(it->second);
                };
                axis_config_[i].micro_steps = static_cast<int>(get("micro_steps", 16.0));
                axis_config_[i].step_angle_deg = get("step_angle_deg", 1.8);
                axis_config_[i].gear_ratio = get("gear_ratio", 1.0);
                axis_config_[i].screw_lead_m = get("screw_lead_m", 0.0);
            }

            const std::string port = info_.hardware_parameters.at("serial_port");
            const int baud = std::stoi(info_.hardware_parameters.at("baudrate"));
            if (!serial_.open(port, baud))
            {
                RCLCPP_ERROR(rclcpp::get_logger("BizonSystemInterface"),
                             "Failed to open serial port '%s' at %d baud", port.c_str(), baud);
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"),
                        "Serial link open on %s at %d baud", port.c_str(), baud);
        }
```

- [ ] **Step 6: Implement `read_serial_sensors` and `write_serial_commands`**

```cpp
    void BizonSystemInterface::read_serial_sensors()
    {
        bizon_protocol::StateFrame st{};
        if (!serial_.receive(st))
        {
            // No fresh frame this cycle. Hold the previous measurement rather
            // than inventing one; a persistent gap is caught by the health check
            // in write_serial_commands().
            serial_link_healthy_ = false;
            return;
        }
        serial_link_healthy_ = true;
        last_state_ = st;

        for (size_t i = 0; i < info_.joints.size() && i < static_cast<size_t>(bizon_protocol::kAxisCount); ++i)
        {
            const auto & cfg = axis_config_[i];
            const double previous = hw_measured_positions_[i];
            hw_measured_positions_[i] =
                (cfg.screw_lead_m > 0.0)
                ? stepsToMetres(st.actual_steps[i], cfg.micro_steps, cfg.step_angle_deg,
                                cfg.gear_ratio, cfg.screw_lead_m)
                : stepsToRadians(st.actual_steps[i], cfg.micro_steps, cfg.step_angle_deg,
                                 cfg.gear_ratio);
            hw_measured_velocities_[i] = hw_measured_positions_[i] - previous;
        }
    }

    void BizonSystemInterface::write_serial_commands()
    {
        bizon_protocol::CommandFrame cmd{};
        cmd.seq = ++tx_seq_;
        cmd.flags = bizon_protocol::kFlagEnable;

        for (size_t i = 0; i < info_.joints.size() && i < static_cast<size_t>(bizon_protocol::kAxisCount); ++i)
        {
            const auto & cfg = axis_config_[i];
            cmd.target_steps[i] =
                (cfg.screw_lead_m > 0.0)
                ? metresToSteps(hw_position_commands_[i], cfg.micro_steps, cfg.step_angle_deg,
                                cfg.gear_ratio, cfg.screw_lead_m)
                : radiansToSteps(hw_position_commands_[i], cfg.micro_steps, cfg.step_angle_deg,
                                 cfg.gear_ratio);
        }

        serial_.send(cmd);
    }
```

**Do not** add an echo path here. `read_rviz_sensors()` assigns
`hw_measured_positions_[i] = hw_position_commands_[i]` — correct for RViz, and the single most
damaging line that could be copied into this branch. Echoing makes tracking perfect by
construction, so JTC never aborts, RecoveryNode never runs, and MoveIt reports success for
motions that crashed.

- [ ] **Step 7: Add the xacro parameters**

In `bizon_description/urdf/bizon_system_interface.xacro`, add to `<hardware>`:

```xml
                <param name="serial_port">${serial_port}</param>
                <param name="baudrate">${baudrate}</param>
```

extend the macro signature with `serial_port:=^|/dev/ttyACM0 baudrate:=^|921600`, and give the
`joint_interface` macro per-axis parameters:

```xml
            <xacro:macro name="joint_interface"
                         params="name micro_steps:=16 step_angle_deg:=1.8 gear_ratio:=1.0 screw_lead_m:=0.0">
                <joint name="${name}">
                    <command_interface name="position"/>
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                    <param name="micro_steps">${micro_steps}</param>
                    <param name="step_angle_deg">${step_angle_deg}</param>
                    <param name="gear_ratio">${gear_ratio}</param>
                    <param name="screw_lead_m">${screw_lead_m}</param>
                </joint>
            </xacro:macro>

            <xacro:joint_interface name="${prefix}rev1"/>
            <xacro:joint_interface name="${prefix}pris1" screw_lead_m="0.002"/>
            <xacro:joint_interface name="${prefix}rev2"/>
            <xacro:joint_interface name="${prefix}rev3"/>
```

The three gripper `joint_interface` lines stay as they are for now; Task 8 collapses them.

- [ ] **Step 8: Run the tests and verify they pass**

Expected: 9 step-math tests PASS, plus the protocol and serial suites still green.

- [ ] **Step 9: Commit**

```bash
git add ros_env/src/bizon_ros2_control ros_env/src/bizon_description/urdf/bizon_system_interface.xacro
git commit -m "feat(hw): add the serial hardware branch with unit-tested step conversion"
```

---

### Task 4: The fault chain — JTC tolerances and error propagation

Makes the hardware's disagreement visible to everything above it. This is the task that pays for
keeping MoveIt.

**Files:**
- Create: `ros_env/src/bizon_ros2_control/test/test_fault_propagation.cpp`
- Modify: `ros_env/src/bizon_ros2_control/src/bizon_system_interface.cpp`
- Modify: `ros_env/src/bizon_player_bringup/params/bizon2_full_ros2_controllers.yaml`

**Interfaces:**
- Consumes: `SerialComm`, `McuStub`, the `serial` branch from Task 3.
- Produces: `read()` returning `hardware_interface::return_type::ERROR` on E-stop, latched fault,
  or a link outage exceeding `link_timeout_cycles`.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_ros2_control/test/test_fault_propagation.cpp`:

```cpp
#include <gtest/gtest.h>
#include "bizon_ros2_control/serial_comm.hpp"
#include "mcu_stub.hpp"

using bizon_ros2_control::SerialComm;
using bizon_ros2_control::testing::McuStub;

namespace
{
// Mirrors the rule enforced in BizonSystemInterface::read_serial_sensors():
// a state frame carrying E-stop or a non-zero fault code is an error, not data.
bool frameIsFault(const bizon_protocol::StateFrame & st)
{
  return (st.status & bizon_protocol::kStatusEstop) ||
         (st.status & bizon_protocol::kStatusFault) ||
         st.fault_code != 0;
}
}  // namespace

TEST(FaultPropagation, HealthyFrameIsNotAFault)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));
  stub.emitState(1);

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_FALSE(frameIsFault(st));
}

TEST(FaultPropagation, EstopStatusIsAFault)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));
  stub.setStatus(bizon_protocol::kStatusEstop);
  stub.emitState(2);

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_TRUE(frameIsFault(st));
}

// The stall bit is what substitutes for the torque limits the URDF does not
// have: every revolute joint carries effort="0.0".
TEST(FaultPropagation, StallBitSurvivesTheRoundTrip)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));
  stub.setStallBits(0b00010);
  stub.emitState(3);

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.stall_bits, 0b00010);
}

// The regression that matters most. A stub that echoes targets makes tracking
// perfect by construction; a stub that does not move reproduces a jammed axis.
// If these two ever produce the same measured position, the echo bug is back
// and every safety mechanism above ros2_control is decorative.
TEST(FaultPropagation, AStuckAxisDoesNotEchoTheCommandedTarget)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  stub.setEchoTargets(false);
  stub.setActualSteps(0, 0);

  bizon_protocol::CommandFrame cmd{};
  cmd.seq = 1;
  cmd.target_steps[0] = 5000;
  ASSERT_TRUE(comm.send(cmd));
  stub.pump();

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.actual_steps[0], 0);
  EXPECT_NE(st.actual_steps[0], cmd.target_steps[0]);
}

TEST(FaultPropagation, EchoModeIsOnlyReachableWhenExplicitlyRequested)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  stub.setEchoTargets(true);
  bizon_protocol::CommandFrame cmd{};
  cmd.seq = 1;
  cmd.target_steps[2] = -1234;
  ASSERT_TRUE(comm.send(cmd));
  stub.pump();

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.actual_steps[2], -1234);
}
```

- [ ] **Step 2: Run it and verify it fails**

Expected: FAIL — `test_fault_propagation` is not a registered target.

- [ ] **Step 3: Register the test**

```cmake
  ament_add_gtest(test_fault_propagation test/test_fault_propagation.cpp src/serial_comm.cpp)
  target_include_directories(test_fault_propagation PRIVATE include test)
  target_link_libraries(test_fault_propagation util)
```

- [ ] **Step 4: Run it and verify it passes**

Expected: 5 tests PASS.

- [ ] **Step 5: Return ERROR from `read()` on a fault**

Extend `read_serial_sensors()` with a latch, and change `read()` so the fault reaches
ros2_control instead of being logged and swallowed:

```cpp
    hardware_interface::return_type BizonSystemInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (this->*read_sensors_)();

        if (serial_fault_latched_)
        {
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }
```

and inside `read_serial_sensors()`, after `last_state_ = st;`:

```cpp
        if ((st.status & bizon_protocol::kStatusEstop) ||
            (st.status & bizon_protocol::kStatusFault) ||
            st.fault_code != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("BizonSystemInterface"),
                         "MCU fault: status=0x%04X fault_code=%u stall_bits=0x%02X",
                         st.status, st.fault_code, st.stall_bits);
            serial_fault_latched_ = true;
        }
```

Add `bool serial_fault_latched_ = false;` to the header and clear it in `on_activate()`. Latching
is deliberate: a stall that clears itself the instant the arm stops must not let the tree
continue as though nothing happened.

Add to the header, next to `serial_fault_latched_`:

```cpp
        int missed_frames_ = 0;
        int link_timeout_cycles_ = 10;   ///< 10 cycles at 100 Hz = 100 ms of silence
```

and in `read_serial_sensors()`, replace the bare `serial_link_healthy_ = false;` early return with:

```cpp
        if (!serial_.receive(st))
        {
            serial_link_healthy_ = false;
            if (++missed_frames_ >= link_timeout_cycles_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("BizonSystemInterface"),
                             "No MCU frame for %d cycles; treating the link as down",
                             missed_frames_);
                serial_fault_latched_ = true;
            }
            return;
        }
        missed_frames_ = 0;
```

An unplugged USB cable must be a fault, not a freeze: without this the last good position is held
forever, tracking looks perfect, and the arm silently stops being controlled.

- [ ] **Step 6: Add the JTC tolerances**

In `bizon_player_bringup/params/bizon2_full_ros2_controllers.yaml`, under
`/bizon2/arm_group_controller: ros__parameters:`:

```yaml
    constraints:
      stopped_velocity_tolerance: 0.02
      goal_time: 0.5
      bizon2rev1:  { trajectory: 0.05,  goal: 0.01  }
      bizon2pris1: { trajectory: 0.005, goal: 0.002 }
      bizon2rev2:  { trajectory: 0.05,  goal: 0.01  }
      bizon2rev3:  { trajectory: 0.10,  goal: 0.02  }
```

The file currently declares no tolerances at all, which means JTC accepts unlimited tracking
error. These are starting values; tighten them once real tracking error is measured on the bench.

- [ ] **Step 7: Run the full package test suite**

Expected: all four suites green.

- [ ] **Step 8: Commit**

```bash
git add ros_env/src/bizon_ros2_control ros_env/src/bizon_player_bringup/params/bizon2_full_ros2_controllers.yaml
git commit -m "feat(hw): propagate MCU faults through read() and give the JTC real tolerances"
```

- [ ] **Step 9: Bench verification (requires hardware)**

With the arm powered and homed, command a slow move and physically block the link's motion by
hand. Expected log order:

```
[BizonSystemInterface] MCU fault: status=... stall_bits=...
[arm_group_controller] Aborted due to path tolerance violation
[ArmPlugin] arm move failed
[RecoveryNode] work branch failed, running recovery (attempt 1/3)
```

**Fail signal:** the move completes normally with the axis blocked. That means measured position
is tracking commanded position rather than the MCU counter — re-read Task 3 Step 6.

---

### Task 5: Homing on activation

**Files:**
- Modify: `ros_env/src/bizon_ros2_control/src/bizon_system_interface.cpp`
- Modify: `ros_env/src/bizon_ros2_control/include/bizon_ros2_control/bizon_system_interface.hpp`
- Create: `ros_env/src/bizon_ros2_control/test/test_homing.cpp`

**Interfaces:**
- Consumes: the `serial` branch, `McuStub` (which sets `homed_bits = 0b11111` on receiving
  `kFlagHomeRequest`).
- Produces: `on_activate()` that blocks until every axis reports homed or a timeout elapses.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_ros2_control/test/test_homing.cpp`:

```cpp
#include <gtest/gtest.h>
#include "bizon_ros2_control/serial_comm.hpp"
#include "mcu_stub.hpp"

using bizon_ros2_control::SerialComm;
using bizon_ros2_control::testing::McuStub;

TEST(Homing, HomeRequestFlagReachesTheMcu)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  bizon_protocol::CommandFrame cmd{};
  cmd.seq = 1;
  cmd.flags = bizon_protocol::kFlagEnable | bizon_protocol::kFlagHomeRequest;
  ASSERT_TRUE(comm.send(cmd));
  stub.pump();

  EXPECT_TRUE(stub.lastCommand().flags & bizon_protocol::kFlagHomeRequest);
}

TEST(Homing, AllAxesReportHomedAfterTheRequest)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  bizon_protocol::CommandFrame cmd{};
  cmd.seq = 1;
  cmd.flags = bizon_protocol::kFlagEnable | bizon_protocol::kFlagHomeRequest;
  ASSERT_TRUE(comm.send(cmd));
  stub.pump();

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.homed_bits, 0b11111);
}

TEST(Homing, AxesAreNotHomedBeforeTheRequest)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));
  stub.emitState(1);

  bizon_protocol::StateFrame st{};
  ASSERT_TRUE(comm.receive(st));
  EXPECT_EQ(st.homed_bits, 0);
}
```

- [ ] **Step 2: Run it and verify it fails**

Expected: FAIL — `test_homing` is not a registered target.

- [ ] **Step 3: Register the test and run it**

```cmake
  ament_add_gtest(test_homing test/test_homing.cpp src/serial_comm.cpp)
  target_include_directories(test_homing PRIVATE include test)
  target_link_libraries(test_homing util)
```

Expected: 3 tests PASS.

- [ ] **Step 4: Implement homing in `on_activate`**

```cpp
    hardware_interface::CallbackReturn BizonSystemInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        serial_fault_latched_ = false;

        if (info_.hardware_parameters.at("ros2_control_hardware_type") != "serial")
        {
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // MoveIt must not plan against an unreferenced arm: with no encoder the
        // step counter is meaningless until the endstops define its zero.
        bizon_protocol::CommandFrame cmd{};
        cmd.seq = ++tx_seq_;
        cmd.flags = bizon_protocol::kFlagEnable | bizon_protocol::kFlagClearFault |
                    bizon_protocol::kFlagHomeRequest;
        if (!serial_.send(cmd))
        {
            RCLCPP_ERROR(rclcpp::get_logger("BizonSystemInterface"), "Failed to send home request");
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
        const uint8_t all_axes = (1u << bizon_protocol::kAxisCount) - 1u;
        while (std::chrono::steady_clock::now() < deadline)
        {
            bizon_protocol::StateFrame st{};
            if (serial_.receive(st) && (st.homed_bits & all_axes) == all_axes)
            {
                last_state_ = st;
                for (size_t i = 0; i < info_.joints.size() && i < static_cast<size_t>(bizon_protocol::kAxisCount); ++i)
                {
                    const auto & cfg = axis_config_[i];
                    hw_measured_positions_[i] =
                        (cfg.screw_lead_m > 0.0)
                        ? stepsToMetres(st.actual_steps[i], cfg.micro_steps, cfg.step_angle_deg,
                                        cfg.gear_ratio, cfg.screw_lead_m)
                        : stepsToRadians(st.actual_steps[i], cfg.micro_steps, cfg.step_angle_deg,
                                         cfg.gear_ratio);
                    // Start commanding from where the arm actually is, so
                    // activation does not order a jump to zero.
                    hw_position_commands_[i] = hw_measured_positions_[i];
                }
                RCLCPP_INFO(rclcpp::get_logger("BizonSystemInterface"), "All axes homed");
                return hardware_interface::CallbackReturn::SUCCESS;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        RCLCPP_ERROR(rclcpp::get_logger("BizonSystemInterface"), "Homing timed out after 60 s");
        return hardware_interface::CallbackReturn::ERROR;
    }
```

Add `#include <chrono>` and `#include <thread>`.

- [ ] **Step 5: Run the full suite and commit**

```bash
git add ros_env/src/bizon_ros2_control
git commit -m "feat(hw): home every axis on activation before any motion is accepted"
```

- [ ] **Step 6: Bench verification (requires hardware)**

Activate the controller with the arm parked anywhere. **Pass:** every axis seeks its endstop,
backs off, and `All axes homed` appears before any controller accepts a goal; the arm does not
lurch toward zero afterwards. **Fail signal:** the arm jumps on activation — `hw_position_commands_`
was not seeded from the homed measurement.

---

### Task 6: Delete the duplicate geometry and call `bizon_chess`

Closes the NaN defect on the path that actually executes. `bizon_chess::worldToJointAngles`
already guards `|d| > 1`; `make_decision_client_node.cpp` still runs its own copy, which feeds
`sqrtf(1 - D*D)` unguarded and emits NaN joint targets for unreachable squares. On hardware that
is a corrupt position command.

**Files:**
- Modify: `ros_env/src/bizon_chess/include/bizon_chess/board_geometry.hpp`
- Modify: `ros_env/src/bizon_chess/src/board_geometry.cpp`
- Modify: `ros_env/src/bizon_chess/test/test_board_geometry.cpp`
- Modify: `ros_env/src/bizon_behavior_clients/plugins/action/make_decision_client_node.cpp`
- Modify: `ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/action/make_decision_client_node.hpp`
- Modify: `ros_env/src/bizon_behavior_clients/CMakeLists.txt`, `package.xml`

**Interfaces:**
- Consumes: `bizon_chess::RobotParams`, `squareToWorld`, `worldToJointAngles` from pre-hardware Task 1.
- Produces: `bool bizon_chess::squareToJointAngles(const std::string & square, const RobotParams & p, double & q1, double & q2)`
  — the mirroring and base-offset step that is currently open-coded eight times in the decision node.

- [ ] **Step 1: Write the failing test**

Append to `ros_env/src/bizon_chess/test/test_board_geometry.cpp`:

```cpp
TEST(BoardGeometry, SquareToJointAnglesSucceedsForAReachableSquare)
{
  bizon_chess::RobotParams p;
  double q1 = 0.0, q2 = 0.0;
  ASSERT_TRUE(bizon_chess::squareToJointAngles("e4", p, q1, q2));
  EXPECT_TRUE(std::isfinite(q1));
  EXPECT_TRUE(std::isfinite(q2));
}

// The whole point: an unreachable target must be reported, never converted into
// a NaN joint command. make_decision_client_node's private copy of this maths
// had no domain guard at all.
TEST(BoardGeometry, UnreachableTargetIsRejectedRatherThanReturningNaN)
{
  bizon_chess::RobotParams p;
  p.link_l1 = 0.05;
  p.link_l2 = 0.05;   // total reach 0.10 m, far short of any square
  double q1 = 0.0, q2 = 0.0;
  EXPECT_FALSE(bizon_chess::squareToJointAngles("a1", p, q1, q2));
}

TEST(BoardGeometry, MirroredParamsGiveADifferentSolutionThanUnmirrored)
{
  bizon_chess::RobotParams white;
  bizon_chess::RobotParams black;
  black.mirrored = true;

  double qw1 = 0.0, qw2 = 0.0, qb1 = 0.0, qb2 = 0.0;
  ASSERT_TRUE(bizon_chess::squareToJointAngles("a1", white, qw1, qw2));
  ASSERT_TRUE(bizon_chess::squareToJointAngles("a1", black, qb1, qb2));
  EXPECT_NE(qw1, qb1);
}

TEST(BoardGeometry, MalformedSquareIsRejected)
{
  bizon_chess::RobotParams p;
  double q1 = 0.0, q2 = 0.0;
  EXPECT_FALSE(bizon_chess::squareToJointAngles("z9", p, q1, q2));
  EXPECT_FALSE(bizon_chess::squareToJointAngles("e", p, q1, q2));
}
```

- [ ] **Step 2: Run it and verify it fails**

```bash
sg docker -c "docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp \
  -v /home/gorkem/Documents/projects/bizon_chess_player:/ws \
  -v /home/gorkem/.cache/bizon-sdd:/cb -w /ws/ros_env \
  docker-bizon_chess_player:latest bash -c \
  'source /opt/ros/humble/setup.bash && colcon build --packages-select bizon_chess \
   --build-base /cb/build --install-base /cb/install'"
```

Expected: FAIL — `squareToJointAngles` is not a member of `bizon_chess`.

- [ ] **Step 3: Implement `squareToJointAngles`**

Declare in `board_geometry.hpp` and define in `board_geometry.cpp`:

```cpp
bool squareToJointAngles(const std::string & square, const RobotParams & p, double & q1, double & q2)
{
  double x = 0.0;
  double y = 0.0;
  if (!squareToWorld(square, p, x, y)) {
    return false;
  }

  // Lifted verbatim from the eight open-coded copies in
  // make_decision_client_node.cpp. The black-side robot faces the board from
  // the opposite edge, so both axes flip before the base offset is applied.
  if (p.mirrored) {
    x = -x - p.robot_base_offset_x;
    y = -y;
  } else {
    x = x - p.robot_base_offset_x;
  }

  return worldToJointAngles(x, y, p, q1, q2);
}
```

- [ ] **Step 4: Run it and verify it passes**

Expected: 4 new tests PASS alongside the existing `bizon_chess` suite.

- [ ] **Step 5: Replace the decision node's private copies**

In `make_decision_client_node.cpp`, delete the file-local `get_box_location_wrt_world` and
`calculate_joint_angles` entirely, `#include "bizon_chess/board_geometry.hpp"`, and replace each
of the eight open-coded blocks with:

```cpp
        if (!bizon_chess::squareToJointAngles(box_from_1_, robot_params_, q0_from_1_, q2_from_1_)) {
            RCLCPP_ERROR(rclcpp::get_logger("MakeDecisionNode"),
                         "Square %s is not reachable with the configured geometry", box_from_1_.c_str());
            return BT::NodeStatus::FAILURE;
        }
```

Delete the eight `const float ... // todo: make this configurable` members from the header and
replace them with `bizon_chess::RobotParams robot_params_;`, populated in Task 7. Note the type
change from `float` to `double` — the joint angle members change with it.

Add `bizon_chess` to `find_package`, `ament_target_dependencies` and `package.xml` for
`bizon_behavior_clients`. The pre-hardware plan's pre-flight scan flagged this exact dependency
gap for tasks T1 to T5 and T1 to T7; this is where it gets closed.

- [ ] **Step 6: Run the behavior-clients suite and verify it passes**

Expected: the existing `test_fen_utils`, `test_recovery_node`, `test_arm_action_client_node` and
`test_chess_game_tree_structure` suites stay green.

- [ ] **Step 7: Commit**

```bash
git add ros_env/src/bizon_chess ros_env/src/bizon_behavior_clients
git commit -m "fix(chess): use the guarded bizon_chess IK and delete the NaN-producing duplicate"
```

---

### Task 7: Calibration parameters

Turns `RobotParams` from compile-time defaults into measured values, which is what makes fixed
direct-drive geometry good enough. Systematic error — link lengths, base offset, board position —
is large but constant; stepper repeatability means removing it once removes it for good.

**Files:**
- Create: `ros_env/src/bizon_player_bringup/params/bizon2_calibration.yaml`
- Modify: `ros_env/src/bizon_behavior_clients/plugins/action/make_decision_client_node.cpp`
- Modify: `ros_env/src/bizon_player_bringup/launch/bizon_player.launch.py`
- Create: `ros_env/src/bizon_chess/test/test_calibration_fit.cpp`
- Modify: `ros_env/src/bizon_chess/include/bizon_chess/board_geometry.hpp`, `src/board_geometry.cpp`, `CMakeLists.txt`

**Interfaces:**
- Consumes: `RobotParams`, `squareToJointAngles`.
- Produces: `bool bizon_chess::fitParamsFromCorners(const std::array<std::string, 4> & squares, const std::array<std::pair<double, double>, 4> & measured_xy, RobotParams & io_params)`.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_chess/test/test_calibration_fit.cpp`:

```cpp
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <utility>
#include "bizon_chess/board_geometry.hpp"

// Feeding back exactly what the nominal model predicts must leave the model
// unchanged. If this drifts, the fit has a bias and every calibration run makes
// the robot worse.
TEST(CalibrationFit, PerfectMeasurementsLeaveParamsUnchanged)
{
  bizon_chess::RobotParams p;
  const std::array<std::string, 4> squares{"a1", "a8", "h1", "h8"};
  std::array<std::pair<double, double>, 4> measured{};
  for (size_t i = 0; i < 4; ++i) {
    double x = 0.0, y = 0.0;
    ASSERT_TRUE(bizon_chess::squareToWorld(squares[i], p, x, y));
    measured[i] = {x, y};
  }

  bizon_chess::RobotParams fitted = p;
  ASSERT_TRUE(bizon_chess::fitParamsFromCorners(squares, measured, fitted));
  EXPECT_NEAR(fitted.box_size, p.box_size, 1e-9);
  EXPECT_NEAR(fitted.robot_base_offset_x, p.robot_base_offset_x, 1e-9);
}

// A board 2 mm per square larger than nominal must be recovered as such.
TEST(CalibrationFit, RecoversAnInflatedBoxSize)
{
  bizon_chess::RobotParams truth;
  truth.box_size = 0.039;

  const std::array<std::string, 4> squares{"a1", "a8", "h1", "h8"};
  std::array<std::pair<double, double>, 4> measured{};
  for (size_t i = 0; i < 4; ++i) {
    double x = 0.0, y = 0.0;
    ASSERT_TRUE(bizon_chess::squareToWorld(squares[i], truth, x, y));
    measured[i] = {x, y};
  }

  bizon_chess::RobotParams fitted;   // starts at the nominal 0.03718857142
  ASSERT_TRUE(bizon_chess::fitParamsFromCorners(squares, measured, fitted));
  EXPECT_NEAR(fitted.box_size, 0.039, 1e-6);
}

TEST(CalibrationFit, DegenerateCornersAreRejected)
{
  bizon_chess::RobotParams p;
  const std::array<std::string, 4> squares{"a1", "a1", "a1", "a1"};
  const std::array<std::pair<double, double>, 4> measured{
    std::make_pair(0.0, 0.0), std::make_pair(0.0, 0.0),
    std::make_pair(0.0, 0.0), std::make_pair(0.0, 0.0)};
  EXPECT_FALSE(bizon_chess::fitParamsFromCorners(squares, measured, p));
}
```

- [ ] **Step 2: Run it and verify it fails**

Expected: FAIL — `fitParamsFromCorners` is not a member of `bizon_chess`.

- [ ] **Step 3: Implement the fit**

`box_size` comes from the measured span between opposite corners divided by the seven square
pitches between them; `robot_base_offset_x` from the mean X residual. Reject the fit when the
corner span is under one nominal square, which is the degenerate case above.

```cpp
bool fitParamsFromCorners(
  const std::array<std::string, 4> & squares,
  const std::array<std::pair<double, double>, 4> & measured_xy,
  RobotParams & io_params)
{
  double min_x = measured_xy[0].first, max_x = min_x;
  double min_y = measured_xy[0].second, max_y = min_y;
  for (const auto & m : measured_xy) {
    min_x = std::min(min_x, m.first);
    max_x = std::max(max_x, m.first);
    min_y = std::min(min_y, m.second);
    max_y = std::max(max_y, m.second);
  }

  const double span_x = max_x - min_x;
  const double span_y = max_y - min_y;
  if (span_x < io_params.box_size || span_y < io_params.box_size) {
    return false;   // corners too close together to constrain anything
  }

  // Seven square pitches separate rank 1 from rank 8 and file a from file h.
  io_params.box_size = 0.5 * (span_x / 7.0 + span_y / 7.0);

  double residual = 0.0;
  for (size_t i = 0; i < squares.size(); ++i) {
    double nx = 0.0, ny = 0.0;
    if (!squareToWorld(squares[i], io_params, nx, ny)) {
      return false;
    }
    residual += nx - measured_xy[i].first;
  }
  io_params.robot_base_offset_x += residual / static_cast<double>(squares.size());
  return true;
}
```

- [ ] **Step 4: Run it and verify it passes**

Expected: 3 tests PASS.

- [ ] **Step 5: Add the calibration file and load it**

`ros_env/src/bizon_player_bringup/params/bizon2_calibration.yaml`:

```yaml
# Measured on the built arm, not derived from CAD. Regenerate after any
# mechanical change: these values are what make fixed direct-drive geometry
# accurate enough to pick a 37.19 mm square.
bizon_behavior_tree_client:
  ros__parameters:
    robot_params:
      box_size: 0.03718857142
      robot_base_offset_x: -0.3
      link_l1: 0.29
      link_l2: 0.18
      gap_eef_close: 0.04
      gap_eef_open: 0.20
      limit_l1_down: 0.155
      limit_l1_up: 0.08
      mirrored: false
```

Declare and read each field in the decision node's constructor into `robot_params_`, and pass
the file to the behavior-tree client node in `bizon_player.launch.py`.

- [ ] **Step 6: Commit**

```bash
git add ros_env/src/bizon_chess ros_env/src/bizon_player_bringup ros_env/src/bizon_behavior_clients
git commit -m "feat(chess): load robot geometry from a calibration file instead of constants"
```

- [ ] **Step 7: Bench procedure (requires hardware)**

Jog the arm so the gripper centres on a1, a8, h1 and h8 in turn, record the four commanded
`(x, y)` pairs, run them through `fitParamsFromCorners`, and write the result into
`bizon2_calibration.yaml`. **Pass:** after reloading, commanding each of the four corners plus e4
puts the gripper visibly centred on the square. **Fail signal:** a consistent offset in one
direction — the base offset did not converge; re-measure with the arm homed first.

---

### Task 8: One gripper motor, three fingers

The URDF declares three finger joints and `hand_group` commands all three. One stepper driving
all three mechanically needs `mimic` joints and a single command interface.

**Files:**
- Modify: `ros_env/src/bizon_description/urdf/bizon2.urdf` and `bizon.urdf.xacro`
- Modify: `ros_env/src/bizon_description/urdf/bizon_system_interface.xacro`
- Modify: `ros_env/src/bizon_player_bringup/params/bizon2_full_ros2_controllers.yaml`
- Modify: `ros_env/src/bizon2_moveit_pkg/config/` — `hand_group` definition
- Create: `ros_env/src/bizon_ros2_control/test/test_gripper_mapping.cpp`

**Interfaces:**
- Consumes: the `serial` branch's axis 4.
- Produces: `hand_group_controller` commanding one joint, `bizon2gripper_finger1_joint`, with
  fingers 2 and 3 mimicking it at multiplier 1.0.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_ros2_control/test/test_gripper_mapping.cpp`:

```cpp
#include <gtest/gtest.h>
#include "bizon_ros2_control/serial_comm.hpp"
#include "mcu_stub.hpp"

using bizon_ros2_control::SerialComm;
using bizon_ros2_control::testing::McuStub;

// The gripper occupies exactly one axis on the wire even though the URDF shows
// three fingers. If a second finger ever claims an axis, the arm axes shift and
// every joint command lands on the wrong motor.
TEST(GripperMapping, GripperOccupiesTheFifthAxisOnly)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  bizon_protocol::CommandFrame cmd{};
  cmd.seq = 1;
  cmd.target_steps[4] = 640;
  ASSERT_TRUE(comm.send(cmd));
  stub.pump();

  EXPECT_EQ(stub.lastCommand().target_steps[4], 640);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(stub.lastCommand().target_steps[i], 0) << "arm axis " << i << " disturbed";
  }
}

// IHOLD_IRUN is the only force limit available: closing at reduced current lets
// the motor stall harmlessly on a rigid piece instead of crushing it. This is
// what closes finding F1.
TEST(GripperMapping, GripperCurrentIsCarriedOnTheWire)
{
  McuStub stub;
  SerialComm comm;
  ASSERT_TRUE(comm.open(stub.devicePath(), 115200));

  bizon_protocol::CommandFrame cmd{};
  cmd.seq = 1;
  cmd.gripper_current = 8;
  ASSERT_TRUE(comm.send(cmd));
  stub.pump();

  EXPECT_EQ(stub.lastCommand().gripper_current, 8);
}
```

- [ ] **Step 2: Run it and verify it fails, then register and pass it**

```cmake
  ament_add_gtest(test_gripper_mapping test/test_gripper_mapping.cpp src/serial_comm.cpp)
  target_include_directories(test_gripper_mapping PRIVATE include test)
  target_link_libraries(test_gripper_mapping util)
```

Expected: 2 tests PASS.

- [ ] **Step 3: Add the mimic joints**

In `bizon2.urdf`, after the `bizon2gripper_finger2_joint` and `bizon2gripper_finger3_joint`
definitions, add inside each:

```xml
    <mimic joint="bizon2gripper_finger1_joint" multiplier="1.0" offset="0.0"/>
```

- [ ] **Step 4: Reduce the controller to one joint**

In `bizon2_full_ros2_controllers.yaml`:

```yaml
/bizon2/hand_group_controller:
  ros__parameters:
    joints:
      - bizon2gripper_finger1_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 1.0
      bizon2gripper_finger1_joint: { trajectory: 0.20, goal: 0.10 }
```

The gripper's tolerances are deliberately loose: a current-limited close is *expected* to stop
short of its commanded position when it meets a piece. Tight tolerances here would turn every
successful grasp into an aborted trajectory.

Remove the finger 2 and 3 `joint_interface` lines from `bizon_system_interface.xacro`, leaving
five ros2_control joints total — matching `bizon_protocol::kAxisCount`.

- [ ] **Step 5: Update the MoveIt `hand_group`**

In `bizon2_moveit_pkg/config/`, reduce `hand_group` to the single finger 1 joint and re-record
the named `open`/`closed` states against it.

- [ ] **Step 6: Run the suites and commit**

```bash
git add ros_env/src/bizon_description ros_env/src/bizon_player_bringup ros_env/src/bizon2_moveit_pkg ros_env/src/bizon_ros2_control
git commit -m "feat(gripper): drive three fingers from one motor via mimic joints"
```

---

### Task 9: Playing a human — FEN debounce and the history file

**Files:**
- Modify: `ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/plugins/board_plugin.hpp`
- Modify: `ros_env/src/bizon_behavior_servers/plugins/board_plugin.cpp`
- Modify: `ros_env/src/bizon_behavior_clients/plugins/action/make_decision_client_node.cpp` and its header
- Create: `ros_env/src/bizon_behavior_servers/test/test_fen_debounce.cpp`
- Modify: `ros_env/src/bizon_player_bringup/params/bizon_behavior_params.yaml`

**Interfaces:**
- Consumes: the FEN string `BoardPlugin` already produces.
- Produces: `bizon_behaviors::FenDebouncer` with `bool accept(const std::string & fen)` returning
  true only once `required_repeats` consecutive identical readings have arrived, and
  `void reset()`. `MakeDecisionNode` gains a `last_moves_path` ROS parameter.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_behavior_servers/test/test_fen_debounce.cpp`:

```cpp
#include <gtest/gtest.h>
#include <string>
#include "bizon_behavior_servers/plugins/fen_debouncer.hpp"

using bizon_behaviors::FenDebouncer;

// A human's hand crosses the board mid-move, so single frames are unreliable.
// Requiring N identical readings is both the perception filter and the safety
// interlock: the arm only moves once the board has been still.
TEST(FenDebounce, RejectsUntilTheRequiredRepeatsArrive)
{
  FenDebouncer d(3);
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/8"));
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/8"));
  EXPECT_TRUE(d.accept("8/8/8/8/8/8/8/8"));
}

TEST(FenDebounce, ADifferentReadingRestartsTheCount)
{
  FenDebouncer d(3);
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/8"));
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/K7"));
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/K7"));
  EXPECT_TRUE(d.accept("8/8/8/8/8/8/8/K7"));
}

TEST(FenDebounce, StaysAcceptedWhileTheBoardIsStill)
{
  FenDebouncer d(2);
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/8"));
  EXPECT_TRUE(d.accept("8/8/8/8/8/8/8/8"));
  EXPECT_TRUE(d.accept("8/8/8/8/8/8/8/8"));
}

TEST(FenDebounce, ResetForcesTheCountToStartOver)
{
  FenDebouncer d(2);
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/8"));
  EXPECT_TRUE(d.accept("8/8/8/8/8/8/8/8"));
  d.reset();
  EXPECT_FALSE(d.accept("8/8/8/8/8/8/8/8"));
}

TEST(FenDebounce, EmptyReadingsNeverAccept)
{
  FenDebouncer d(2);
  EXPECT_FALSE(d.accept(""));
  EXPECT_FALSE(d.accept(""));
  EXPECT_FALSE(d.accept(""));
}

TEST(FenDebounce, ARequirementOfOneAcceptsImmediately)
{
  FenDebouncer d(1);
  EXPECT_TRUE(d.accept("8/8/8/8/8/8/8/8"));
}
```

- [ ] **Step 2: Run it and verify it fails**

Expected: FAIL — `fen_debouncer.hpp: No such file or directory`.

- [ ] **Step 3: Implement the debouncer**

`ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/plugins/fen_debouncer.hpp`:

```cpp
#ifndef BIZON_BEHAVIOR_SERVERS__PLUGINS__FEN_DEBOUNCER_HPP_
#define BIZON_BEHAVIOR_SERVERS__PLUGINS__FEN_DEBOUNCER_HPP_

#include <string>

namespace bizon_behaviors
{

/// Accepts a board reading only after it has repeated `required_repeats` times.
///
/// Free of ROS and of the plugin so it can be unit tested without a graph.
class FenDebouncer
{
public:
  explicit FenDebouncer(int required_repeats = 3)
  : required_(required_repeats < 1 ? 1 : required_repeats) {}

  bool accept(const std::string & fen)
  {
    if (fen.empty()) {
      count_ = 0;
      last_.clear();
      return false;
    }
    if (fen != last_) {
      last_ = fen;
      count_ = 1;
    } else if (count_ < required_) {
      ++count_;
    }
    return count_ >= required_;
  }

  void reset()
  {
    count_ = 0;
    last_.clear();
  }

private:
  int required_;
  int count_ = 0;
  std::string last_;
};

}  // namespace bizon_behaviors

#endif  // BIZON_BEHAVIOR_SERVERS__PLUGINS__FEN_DEBOUNCER_HPP_
```

- [ ] **Step 4: Run it and verify it passes**

Expected: 6 tests PASS.

- [ ] **Step 5: Wire it into `BoardPlugin`**

Hold a `FenDebouncer` member, construct it from a new `board_action.fen_required_repeats`
parameter (default 3), and return the action result only once `accept()` is true; keep reporting
RUNNING otherwise.

- [ ] **Step 6: Parameterise the move-history path**

`last_moves_path` is hardcoded to `/home/user/Documents/bizon_chess_player`, a container path
that does not exist on the Jetson. In `make_decision_client_node.hpp`, change the member to:

```cpp
        std::string last_moves_path;   // resolved in the constructor
```

add two ports to `providedPorts()`:

```cpp
                BT::InputPort<std::string>("last_moves_dir",
                    "Directory holding <side>_last_moves.txt; defaults to $BIZON_STATE_DIR or /tmp"),
                BT::InputPort<bool>("new_game",
                    "Truncate the move history on the first tick instead of resuming it"),
```

and replace the constructor's path setup:

```cpp
        std::string dir;
        if (!getInput("last_moves_dir", dir) || dir.empty())
        {
            const char * env = std::getenv("BIZON_STATE_DIR");
            dir = (env != nullptr && env[0] != '\0') ? env : "/tmp";
        }
        last_moves_path = dir + "/" + player_side_ + "_last_moves.txt";

        bool new_game = false;
        getInput("new_game", new_game);
        if (new_game && std::filesystem::exists(last_moves_path))
        {
            // A stale history starts the next game from the previous game's
            // position, and the recovery ladder then searches forward from a
            // board state that no longer exists.
            std::filesystem::remove(last_moves_path);
            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                        "new_game set: cleared %s", last_moves_path.c_str());
        }
```

Pass `new_game="true"` from `chess_game.xml` only when the operator starts a fresh game; the
default of false preserves today's resume-after-restart behaviour.

- [ ] **Step 7: Add the parameter and commit**

```yaml
    board_action:
      plugin: "bizon_behaviors/BoardPlugin"
      fen_required_repeats: 3
```

```bash
git add ros_env/src/bizon_behavior_servers ros_env/src/bizon_behavior_clients ros_env/src/bizon_player_bringup
git commit -m "feat(play): debounce board readings and make the move history path configurable"
```

- [ ] **Step 8: Bench verification (requires hardware)**

Play three human moves against the arm. **Pass:** the arm never starts while a hand is over the
board, and each of its moves begins only after the board has been still. **Fail signal:** the arm
starts moving while the hand is still in frame — raise `fen_required_repeats`.

---

### Task 10: Periodic re-homing

Spec §3 requires it and nothing else in this plan provides it. With no encoder, a skipped step is
a permanent position error that accumulates silently across a game. Homing costs seconds; a chess
robot has minutes between moves. This is the only defence.

**Files:**
- Create: `ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/condition/needs_rehome_node.hpp`
- Create: `ros_env/src/bizon_behavior_clients/plugins/condition/needs_rehome_node.cpp`
- Create: `ros_env/src/bizon_behavior_clients/test/test_needs_rehome_node.cpp`
- Modify: `ros_env/src/bizon_behavior_clients/CMakeLists.txt`, `src/main.cpp`
- Modify: `ros_env/src/bizon_behavior_clients/behavior_trees/chess_game.xml`

**Interfaces:**
- Consumes: nothing from earlier tasks; the homing action itself is the `on_activate` path from Task 5,
  reached by deactivating and reactivating the hardware component.
- Produces: BT node `NeedsRehome` with input port `interval` (moves between re-homes, default 10)
  returning SUCCESS when a re-home is due and FAILURE otherwise.

- [ ] **Step 1: Write the failing test**

`ros_env/src/bizon_behavior_clients/test/test_needs_rehome_node.cpp`:

```cpp
#include <gtest/gtest.h>
#include "behaviortree_cpp/bt_factory.h"
#include "bizon_behavior_clients/plugins/condition/needs_rehome_node.hpp"

namespace
{
BT::NodeConfiguration makeConfig(BT::Blackboard::Ptr bb, int interval)
{
  BT::NodeConfiguration cfg;
  cfg.blackboard = bb;
  cfg.input_ports["interval"] = std::to_string(interval);
  return cfg;
}
}  // namespace

TEST(NeedsRehome, FirstTickIsDueSoTheArmStartsReferenced)
{
  auto bb = BT::Blackboard::create();
  bizon_behavior_clients::NeedsRehomeNode node("NeedsRehome", makeConfig(bb, 3));
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

TEST(NeedsRehome, NotDueAgainUntilTheIntervalElapses)
{
  auto bb = BT::Blackboard::create();
  bizon_behavior_clients::NeedsRehomeNode node("NeedsRehome", makeConfig(bb, 3));
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(node.tick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

TEST(NeedsRehome, AnIntervalOfOneIsDueEveryTick)
{
  auto bb = BT::Blackboard::create();
  bizon_behavior_clients::NeedsRehomeNode node("NeedsRehome", makeConfig(bb, 1));
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}

// A zero or negative interval must not divide by zero or disable homing
// silently; clamp to 1 so the safe behaviour is the degenerate one.
TEST(NeedsRehome, NonPositiveIntervalClampsToEveryTick)
{
  auto bb = BT::Blackboard::create();
  bizon_behavior_clients::NeedsRehomeNode node("NeedsRehome", makeConfig(bb, 0));
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(node.tick(), BT::NodeStatus::SUCCESS);
}
```

- [ ] **Step 2: Run it and verify it fails**

```bash
sg docker -c "docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp \
  -v /home/gorkem/Documents/projects/bizon_chess_player:/ws \
  -v /home/gorkem/.cache/bizon-sdd:/cb -w /ws/ros_env \
  docker-bizon_chess_player:latest bash -c \
  'source /opt/ros/humble/setup.bash && colcon build --packages-select bizon_behavior_clients \
   --build-base /cb/build --install-base /cb/install'"
```

Expected: FAIL — `needs_rehome_node.hpp: No such file or directory`.

- [ ] **Step 3: Write the node**

`ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/condition/needs_rehome_node.hpp`:

```cpp
#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__NEEDS_REHOME_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__NEEDS_REHOME_NODE_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace bizon_behavior_clients
{

/// SUCCESS when a re-home is due, FAILURE otherwise.
///
/// The arm is open loop: a skipped step is a permanent error that no sensor
/// reports and that accumulates over a game. Re-homing on the endstops is the
/// only way to clear it, and the minutes between moves make it free.
class NeedsRehomeNode : public BT::ConditionNode
{
public:
  NeedsRehomeNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  BT::NodeStatus tick() override
  {
    int interval = 10;
    getInput("interval", interval);
    if (interval < 1) {
      interval = 1;
    }

    const bool due = (moves_since_home_ % interval) == 0;
    ++moves_since_home_;
    return due ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("interval", 10, "Moves between re-homes")};
  }

private:
  int moves_since_home_ = 0;
};

}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__NEEDS_REHOME_NODE_HPP_
```

`ros_env/src/bizon_behavior_clients/plugins/condition/needs_rehome_node.cpp`:

```cpp
#include "bizon_behavior_clients/plugins/condition/needs_rehome_node.hpp"

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bizon_behavior_clients::NeedsRehomeNode>("NeedsRehome");
}
```

- [ ] **Step 4: Register the library and the test**

In `bizon_behavior_clients/CMakeLists.txt`, add a `needs_rehome_node` shared library following the
pattern of the existing plugin libraries, append it to `plugin_libs`, add its name to
`plugin_lib_names_` in `src/main.cpp`, and register the gtest:

```cmake
  ament_add_gtest(test_needs_rehome_node test/test_needs_rehome_node.cpp)
  target_include_directories(test_needs_rehome_node PRIVATE include)
  ament_target_dependencies(test_needs_rehome_node behaviortree_cpp)
```

- [ ] **Step 5: Run the tests and verify they pass**

Expected: 4 tests PASS, and the existing `bizon_behavior_clients` suites stay green.

- [ ] **Step 6: Insert the re-home branch into the tree**

In `chess_game.xml`, inside `PlayUntilGameOver` and before the `RecoveryNode`, add:

```xml
                <Fallback name="RehomeIfDue">
                    <Inverter>
                        <NeedsRehome interval="10"/>
                    </Inverter>
                    <ArmActionClient player_side="{player_side}"
                        target_joint_positions="{home_position}"
                        target_hand_position="{hand_close_position}"
                        rehome="true" />
                </Fallback>
```

`ArmActionClient` gains a `rehome` port that sets `kFlagHomeRequest` on the goal. The `Fallback`
plus `Inverter` shape means a move that is not due to re-home costs one condition tick and
nothing else — the same reason `MoveOrWaitForOpponent` is built that way.

- [ ] **Step 7: Update the tree structure test**

`test_chess_game_tree_structure.cpp` asserts the shape of the tree. Add an assertion that
`RehomeIfDue` sits inside `PlayUntilGameOver` and **outside** `RecoveryNode` — a re-home is
scheduled maintenance, not a fault, and putting it in the work branch would spend a retry on it.
Verify the new assertion fails against the previous XML before committing.

- [ ] **Step 8: Commit**

```bash
git add ros_env/src/bizon_behavior_clients
git commit -m "feat(hw): re-home on the endstops every N moves to clear accumulated step loss"
```

- [ ] **Step 9: Bench verification (requires hardware)**

Play a twelve-move game. **Pass:** a homing cycle runs before moves 1 and 11 and at no other
time, and placement accuracy at move 12 is visibly the same as at move 1. **Fail signal:** pieces
drift progressively off-centre across the game — the interval is too long, or homing is not
resetting the step counters.

---

## Prerequisite before first power-on

Task 6 of `docs/superpowers/plans/2026-08-19-pre-hardware-refactor.md` — the lifecycle manager and
the E-stop path — is not optional for physical operation. Today the manager handles one hardcoded
node (`lifecycle_manager.cpp:33-39`) and `LifecycleManagerClient` is never constructed, so there
is no software path that can deactivate the stack. Land it before the arm is energised for the
first time.

The hardware E-stop must cut TMC2209 `EN` in copper, independently of firmware. And settle the Z
axis before the mechanics are finalised: with every revolute joint on a vertical axis, gravity
loads only `bizon2pris1`, so disabling the drivers drops it. A 2 mm-lead leadscrew self-locks; an
8 mm lead does not.
