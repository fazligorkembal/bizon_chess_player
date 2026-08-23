# Pre-Hardware Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the bizon_chess_player stack safe and structurally ready for real hardware on a Jetson, while everything is still verifiable in Isaac Sim.

**Architecture:** Move all long-running work (MoveIt motion, Stockfish decisions) out of the behavior-tree client process and behind lifecycle-managed action servers, following the existing `BoardPlugin` / `TimedBehavior` pattern. Replace the restart-from-zero control node with an explicit recovery subtree that always releases the gripper and retreats before re-planning. Make the lifecycle manager actually manage every server so it can serve as the E-stop path on real hardware.

**Tech Stack:** ROS 2 Humble, BehaviorTree.CPP v4, MoveIt2, ros2_control, pluginlib, Isaac Sim 5.1.0, TensorRT, Stockfish (UCI over pipe), ament_cmake + gtest.

**Spec:** This plan implements the architecture review recorded in this repository's analysis session (2026-08-19). The findings it argues from are restated in "Findings Being Fixed" below, so the plan is self-contained.

---

## Global Constraints

- ROS 2 distro: **Humble**. C++ standard: **17** for `bizon_behavior_clients`, **20** for `bizon_behavior_servers` (already set in its `CMakeLists.txt:4`).
- Namespaces in use: `/bizon2` (white), `/bizon3` (black). Namespace handling stays as-is in this plan; full abstraction is explicitly out of scope.
- All new action definitions go in `bizon_msgs` and must be added to the `rosidl_generate_interfaces` list in `bizon_msgs/CMakeLists.txt`.
- All new behavior plugins follow the existing pattern: derive from `bizon_behaviors::TimedBehavior<ActionT>`, register in `bizon_behavior_servers/behavior_plugin.xml`, add to `behavior_plugins` in `bizon_player_bringup/params/bizon_behavior_params.yaml` **and** `bizon_behavior_params_bizon3.yaml`.
- All new BT client nodes derive from `bizon_behavior_clients::BtActionClientNode<ActionT>` and are registered as separate shared libraries in `bizon_behavior_clients/CMakeLists.txt`, appended to `plugin_libs`, and added to `plugin_lib_names_` in `bizon_behavior_clients/src/main.cpp:30-38`.
- Build command from `ros_env/`: `colcon build --symlink-install --packages-up-to <pkg>` then `. install/setup.bash`.
- Test command: `colcon test --packages-select <pkg> && colcon test-result --verbose`.
- **No behavior may be changed in a task without a test or an explicit sim verification step in that same task.**
- Do not touch `ros_env/src/BehaviorTree.CPP/` or `Stockfish/` — they are upstream clones.

## Findings Being Fixed

Each task cites the finding it closes.

| # | Finding | Evidence |
|---|---------|----------|
| F1 | Gripper stall on a rigid piece reports `FollowJointTrajectory` failure, and the failure path drags the piece across the board | `bizon2_full_ros2_controllers.yaml` hand controller has `command_interfaces: [position]` only; `run_until_success_node.cpp:53` resets to child 0; child 0 is `home + hand_close` |
| F2 | Arm and gripper `move()` calls run concurrently | `arm_action_client_node.cpp:78-86` — two `std::async` launches |
| F3 | MoveIt lives inside the BT process, unmanaged and not cancellable | `arm_action_client_node.cpp:22-23` |
| F4 | Stockfish is `fork()`ed from a BT node constructor with untimed blocking pipe reads in a `SyncActionNode` | `make_decision_client_node.cpp:130-165` |
| F5 | Lifecycle manager manages exactly one hardcoded node; `LifecycleManagerClient` is never instantiated | `lifecycle_manager.cpp:33-39`; grep shows no construction site |
| F6 | Planning scene has no board or pieces, so joint-space plans sweep through occupied squares | no `PlanningSceneInterface` usage anywhere in the repo |
| F7 | Board geometry, link lengths, and robot base offset are compile-time constants | `make_decision_client_node.hpp:103-110` |

---

## File Structure

**New package** `bizon_chess` — pure, ROS-free chess/geometry logic so it can be unit tested without a running graph.

```
bizon_chess/
  include/bizon_chess/board_geometry.hpp   # square -> world XY, world XY -> joint angles
  include/bizon_chess/fen.hpp              # FEN parsing/comparison helpers
  include/bizon_chess/robot_params.hpp     # struct holding what are today the hardcoded constants
  src/board_geometry.cpp
  src/fen.cpp
  test/test_board_geometry.cpp
  test/test_fen.cpp
```

**New interfaces** in `bizon_msgs`:
```
action/Arm.action        # one arm+gripper motion, serialized, with a phase in feedback
action/Decision.action   # FEN in, move plan out
```

**New server plugins** in `bizon_behavior_servers`:
```
plugins/arm_plugin.cpp                              # owns MoveGroupInterface
include/bizon_behavior_servers/plugins/arm_plugin.hpp
plugins/decision_plugin.cpp                         # owns the Stockfish subprocess
include/bizon_behavior_servers/plugins/decision_plugin.hpp
src/stockfish_process.cpp                           # timed UCI subprocess wrapper
include/bizon_behavior_servers/stockfish_process.hpp
```

**New BT client nodes** in `bizon_behavior_clients`:
```
plugins/action/arm_action_client_node.cpp       # REWRITTEN as BtActionClientNode<Arm>
plugins/action/decision_action_client_node.cpp  # replaces make_decision_client_node
plugins/control/recovery_node.cpp               # replaces run_until_success_node
plugins/condition/is_system_active_node.cpp     # lifecycle manager gate
behavior_trees/chess_game.xml                   # new main tree with recovery
```

**Retired at the end** (kept until their replacement is verified): `run_until_success_node.*`, `make_decision_client_node.*`.

---

## Task Ordering Rationale

Task 1 is a prerequisite — it creates the test harness everything else uses. Task 2 is a five-line safety fix worth landing on its own. Task 3 (recovery) comes before the action-server refactors because it is the single largest safety win and the tree XML it produces survives the later refactors unchanged. Tasks 4 and 5 then move MoveIt and Stockfish behind servers. Task 6 wires the lifecycle manager to everything that now exists. Task 7 needs the arm server from Task 4.

---

### Task 1: `bizon_chess` pure-logic package with tests

Closes F7 (partially — makes the constants injectable). Creates the gtest harness the rest of the plan needs.

**Files:**
- Create: `ros_env/src/bizon_chess/CMakeLists.txt`
- Create: `ros_env/src/bizon_chess/package.xml`
- Create: `ros_env/src/bizon_chess/include/bizon_chess/robot_params.hpp`
- Create: `ros_env/src/bizon_chess/include/bizon_chess/board_geometry.hpp`
- Create: `ros_env/src/bizon_chess/src/board_geometry.cpp`
- Test: `ros_env/src/bizon_chess/test/test_board_geometry.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `bizon_chess::RobotParams` (struct), `bizon_chess::squareToWorld(const std::string& square, const RobotParams&, double& x, double& y)`, `bizon_chess::worldToJointAngles(double x, double y, const RobotParams&, double& q1, double& q2)`. Tasks 5 and 7 consume these.

- [ ] **Step 1: Create the package skeleton**

`ros_env/src/bizon_chess/package.xml`:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>bizon_chess</name>
  <version>0.0.0</version>
  <description>Pure chess board geometry and FEN logic, no ROS dependencies</description>
  <maintainer email="bal.gorkem@ustunova.com.tr">Fazli Gorkem Bal</maintainer>
  <license>Proprietary</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`ros_env/src/bizon_chess/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(bizon_chess)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/board_geometry.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include/)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_board_geometry test/test_board_geometry.cpp)
  target_link_libraries(test_board_geometry ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
```

- [ ] **Step 2: Write the failing test**

`ros_env/src/bizon_chess/test/test_board_geometry.cpp`:
```cpp
#include <gtest/gtest.h>
#include <cmath>
#include "bizon_chess/board_geometry.hpp"

using bizon_chess::RobotParams;
using bizon_chess::squareToWorld;
using bizon_chess::worldToJointAngles;

namespace {
RobotParams defaultParams()
{
  RobotParams p;
  p.box_size = 0.03718857142;
  p.robot_base_offset_x = -0.3;
  p.link_l1 = 0.29;
  p.link_l2 = 0.18;
  return p;
}
}  // namespace

// The board is centred on the world origin, so the four centre squares
// straddle it symmetrically at +/- half a square.
TEST(BoardGeometry, CentreSquaresStraddleOrigin)
{
  const auto p = defaultParams();
  double xd4, yd4, xe5, ye5;
  ASSERT_TRUE(squareToWorld("d4", p, xd4, yd4));
  ASSERT_TRUE(squareToWorld("e5", p, xe5, ye5));
  EXPECT_NEAR(xd4, -0.5 * p.box_size, 1e-9);
  EXPECT_NEAR(xe5, 0.5 * p.box_size, 1e-9);
  EXPECT_NEAR(yd4, 0.5 * p.box_size, 1e-9);
  EXPECT_NEAR(ye5, -0.5 * p.box_size, 1e-9);
}

// Corners must sit 3.5 squares from the centre on both axes.
TEST(BoardGeometry, CornersAreSevenHalfSquaresApart)
{
  const auto p = defaultParams();
  double xa1, ya1, xh8, yh8;
  ASSERT_TRUE(squareToWorld("a1", p, xa1, ya1));
  ASSERT_TRUE(squareToWorld("h8", p, xh8, yh8));
  EXPECT_NEAR(xh8 - xa1, 7.0 * p.box_size, 1e-9);
  EXPECT_NEAR(ya1 - yh8, 7.0 * p.box_size, 1e-9);
}

TEST(BoardGeometry, RejectsMalformedSquare)
{
  const auto p = defaultParams();
  double x, y;
  EXPECT_FALSE(squareToWorld("", p, x, y));
  EXPECT_FALSE(squareToWorld("j1", p, x, y));
  EXPECT_FALSE(squareToWorld("a9", p, x, y));
  EXPECT_FALSE(squareToWorld("a", p, x, y));
}

// Forward kinematics of the 2-link solution must land back on the request.
TEST(BoardGeometry, JointAnglesReproduceTargetPosition)
{
  const auto p = defaultParams();
  const double x = 0.30;
  const double y = 0.05;
  double q1, q2;
  ASSERT_TRUE(worldToJointAngles(x, y, p, q1, q2));

  const double fx = p.link_l1 * std::cos(q1) + p.link_l2 * std::cos(q1 + q2);
  const double fy = p.link_l1 * std::sin(q1) + p.link_l2 * std::sin(q1 + q2);
  EXPECT_NEAR(fx, x, 1e-6);
  EXPECT_NEAR(fy, y, 1e-6);
}

// Out of reach must be reported, not silently produce NaN.
TEST(BoardGeometry, RejectsUnreachableTarget)
{
  const auto p = defaultParams();
  double q1, q2;
  EXPECT_FALSE(worldToJointAngles(10.0, 10.0, p, q1, q2));
  EXPECT_FALSE(worldToJointAngles(0.0, 0.0, p, q1, q2));
}
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
cd ros_env
colcon build --packages-select bizon_chess
```
Expected: FAIL — `bizon_chess/board_geometry.hpp: No such file or directory`.

- [ ] **Step 4: Write the headers**

`ros_env/src/bizon_chess/include/bizon_chess/robot_params.hpp`:
```cpp
#ifndef BIZON_CHESS__ROBOT_PARAMS_HPP_
#define BIZON_CHESS__ROBOT_PARAMS_HPP_

namespace bizon_chess
{
/// Everything that was previously a compile-time constant in
/// make_decision_client_node.hpp:103-110. Populated from ROS parameters at
/// runtime so the values can be replaced by calibration on real hardware.
struct RobotParams
{
  double box_size = 0.03718857142;   ///< edge length of one board square [m]
  double robot_base_offset_x = -0.3; ///< arm base X in the board frame [m]
  double link_l1 = 0.29;             ///< proximal link length [m]
  double link_l2 = 0.18;             ///< distal link length [m]
  double gap_eef_close = 0.04;       ///< gripper closed finger gap [m]
  double gap_eef_open = 0.20;        ///< gripper open finger gap [m]
  double limit_l1_down = 0.155;      ///< prismatic travel when placing [m]
  double limit_l1_up = 0.08;         ///< prismatic travel when clear [m]
  bool mirrored = false;             ///< true for the black-side robot
};
}  // namespace bizon_chess
#endif  // BIZON_CHESS__ROBOT_PARAMS_HPP_
```

`ros_env/src/bizon_chess/include/bizon_chess/board_geometry.hpp`:
```cpp
#ifndef BIZON_CHESS__BOARD_GEOMETRY_HPP_
#define BIZON_CHESS__BOARD_GEOMETRY_HPP_

#include <string>
#include "bizon_chess/robot_params.hpp"

namespace bizon_chess
{
/// Convert an algebraic square ("e4") to board-frame XY in metres.
/// Returns false if the square is malformed.
bool squareToWorld(const std::string & square, const RobotParams & p, double & x, double & y);

/// Two-link planar inverse kinematics, elbow-up solution.
/// Returns false if the target is outside the annulus the arm can reach.
bool worldToJointAngles(double x, double y, const RobotParams & p, double & q1, double & q2);
}  // namespace bizon_chess
#endif  // BIZON_CHESS__BOARD_GEOMETRY_HPP_
```

- [ ] **Step 5: Write the implementation**

`ros_env/src/bizon_chess/src/board_geometry.cpp`:
```cpp
#include "bizon_chess/board_geometry.hpp"

#include <cmath>

namespace bizon_chess
{

bool squareToWorld(const std::string & square, const RobotParams & p, double & x, double & y)
{
  if (square.size() != 2) {
    return false;
  }
  const char file_c = square[0];
  const char rank_c = square[1];
  if (file_c < 'a' || file_c > 'h' || rank_c < '1' || rank_c > '8') {
    return false;
  }

  const int col = file_c - 'a';  // 0..7
  const int row = rank_c - '1';  // 0..7

  // Same convention as the original get_box_location_wrt_world():
  // rank runs along +X, file runs along -Y, board centred on the origin.
  x = -3.5 * p.box_size + row * p.box_size;
  y = 3.5 * p.box_size - col * p.box_size;
  return true;
}

bool worldToJointAngles(double x, double y, const RobotParams & p, double & q1, double & q2)
{
  const double r2 = x * x + y * y;
  const double denom = 2.0 * p.link_l1 * p.link_l2;
  if (denom == 0.0) {
    return false;
  }

  const double d = (r2 - p.link_l1 * p.link_l1 - p.link_l2 * p.link_l2) / denom;
  // |d| > 1 means the target is outside the reachable annulus. The original
  // code fed this straight into sqrtf(1 - d*d) and produced NaN joint targets.
  if (!(d >= -1.0 && d <= 1.0)) {
    return false;
  }

  q2 = std::atan2(std::sqrt(1.0 - d * d), d);
  q1 = std::atan2(y, x) - std::atan2(p.link_l2 * std::sin(q2), p.link_l1 + p.link_l2 * std::cos(q2));
  return true;
}

}  // namespace bizon_chess
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
cd ros_env
colcon build --packages-select bizon_chess && colcon test --packages-select bizon_chess && colcon test-result --verbose
```
Expected: 5 tests, all PASS.

- [ ] **Step 7: Commit**

```bash
git add ros_env/src/bizon_chess
git commit -m "feat(bizon_chess): add tested board geometry with injectable robot params"
```

---

### Task 2: Serialize arm and gripper motion

Closes F2. Small, self-contained, verifiable in sim.

**Files:**
- Modify: `ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/action/arm_action_client_node.hpp`
- Modify: `ros_env/src/bizon_behavior_clients/plugins/action/arm_action_client_node.cpp:45-135`

**Interfaces:**
- Consumes: nothing new.
- Produces: no signature change — `ArmActionClient` keeps the same BT ports so `simple_wait_tree.xml` is untouched. Task 4 replaces this file entirely; this task exists so the sim is safe to run in the meantime.

- [ ] **Step 1: Add the phase member to the header**

In `arm_action_client_node.hpp`, inside the class's private section, replace the `move_started_` declaration with:
```cpp
    enum class Phase
    {
      IDLE,
      ARM_MOVING,
      HAND_MOVING,
    };
    Phase phase_{Phase::IDLE};
```

- [ ] **Step 2: Rewrite `onStart` to launch the arm only**

Replace `arm_action_client_node.cpp:74-91` (from `move_group_arm_ptr_->setJointValueTarget(...)` through `return BT::NodeStatus::RUNNING;`) with:
```cpp
        move_group_arm_ptr_->setJointValueTarget(target_joint_positions_);

        // Arm first, gripper afterwards. Running them concurrently means a
        // settling correction on the arm can execute while the fingers are
        // closing, which knocks the piece over on real hardware.
        move_future_arm_ = std::async(std::launch::async, [this]()
        {
            return move_group_arm_ptr_->move();
        });

        phase_ = Phase::ARM_MOVING;
        RCLCPP_INFO(node_->get_logger(), "Arm movement started");

        return BT::NodeStatus::RUNNING;
```

- [ ] **Step 3: Rewrite `onRunning` as a two-phase state machine**

Replace the whole body of `ArmActionClientNode::onRunning()` (`arm_action_client_node.cpp:94-125`) with:
```cpp
    BT::NodeStatus ArmActionClientNode::onRunning()
    {
        if (phase_ == Phase::ARM_MOVING)
        {
            if (move_future_arm_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
            {
                return BT::NodeStatus::RUNNING;
            }

            const auto result_arm = move_future_arm_.get();
            if (result_arm != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Arm movement failed with error code: %d", result_arm.val);
                phase_ = Phase::IDLE;
                return BT::NodeStatus::FAILURE;
            }

            // Arm has settled; only now command the gripper.
            move_group_hand_ptr_->setJointValueTarget(target_hand_position_);
            move_future_hand_ = std::async(std::launch::async, [this]()
            {
                return move_group_hand_ptr_->move();
            });
            phase_ = Phase::HAND_MOVING;
            return BT::NodeStatus::RUNNING;
        }

        if (phase_ == Phase::HAND_MOVING)
        {
            if (move_future_hand_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready)
            {
                return BT::NodeStatus::RUNNING;
            }

            const auto result_hand = move_future_hand_.get();
            phase_ = Phase::IDLE;
            if (result_hand != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Hand movement failed with error code: %d", result_hand.val);
                return BT::NodeStatus::FAILURE;
            }

            RCLCPP_INFO(node_->get_logger(), "Arm and hand movement completed successfully");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }
```

- [ ] **Step 4: Update `onHalted`**

Replace `arm_action_client_node.cpp:127-135` with:
```cpp
    void ArmActionClientNode::onHalted()
    {
        RCLCPP_WARN(node_->get_logger(), "ArmActionClient halted - stopping movement");
        if (phase_ == Phase::ARM_MOVING)
        {
            move_group_arm_ptr_->stop();
        }
        else if (phase_ == Phase::HAND_MOVING)
        {
            move_group_hand_ptr_->stop();
        }
        phase_ = Phase::IDLE;
    }
```

- [ ] **Step 5: Build**

```bash
cd ros_env
colcon build --packages-select bizon_behavior_clients
```
Expected: build succeeds with no reference to `move_started_` remaining.

- [ ] **Step 6: Verify in simulation**

Terminal A: `./isaac_sim_standalone.sh`
Terminal B–D (in container):
```bash
ros2 launch bizon_player_bringup bizon_player_bringup.launch.py prefix:=bizon2
ros2 launch bizon_player_bringup bizon_lifecycle_dev.launch.py namespace:=bizon2
ros2 run bizon_behavior_clients bizon_behavior_tree_client_main --ros-args -r __ns:=/bizon2 -p use_sim_time:=true
```
Expected in the log: for each `MoveSequence` step, `"Arm movement started"` is followed by the gripper log only after the arm future resolves — never interleaved. One full white move completes and the piece ends on the target square.

- [ ] **Step 7: Commit**

```bash
git add ros_env/src/bizon_behavior_clients/plugins/action/arm_action_client_node.cpp \
        ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/action/arm_action_client_node.hpp
git commit -m "fix(arm): serialize arm and gripper motion to avoid disturbing pieces mid-grasp"
```

---

### Task 3: Recovery control node and recovery subtree

Closes F1. The largest safety win in the plan.

**Files:**
- Create: `ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/control/recovery_node.hpp`
- Create: `ros_env/src/bizon_behavior_clients/plugins/control/recovery_node.cpp`
- Create: `ros_env/src/bizon_behavior_clients/behavior_trees/chess_game.xml`
- Modify: `ros_env/src/bizon_behavior_clients/CMakeLists.txt` (add `recovery_node` library)
- Modify: `ros_env/src/bizon_behavior_clients/src/main.cpp:30-38` (register plugin) and `:44` (load `chess_game.xml`)
- Test: `ros_env/src/bizon_behavior_clients/test/test_recovery_node.cpp`

**Interfaces:**
- Consumes: `ArmActionClient` ports from Task 2.
- Produces: BT node `RecoveryNode` with exactly two children — child 0 is the work branch, child 1 is the recovery branch — and an input port `number_of_retries` (int, default 3). Task 6 nests its `IsSystemActive` gate above this node.

**Semantics.** Tick child 0. On SUCCESS, return SUCCESS. On FAILURE, if retries remain, tick child 1 (recovery) to completion; if recovery succeeds, increment the retry counter, halt child 0 and return RUNNING so child 0 restarts from the top on the next tick. If recovery fails, or retries are exhausted, return FAILURE. This is the contract `run_until_success_node.cpp` was missing: **failure never re-enters the work branch without the recovery branch running first.**

- [ ] **Step 1: Write the failing test**

Add to `ros_env/src/bizon_behavior_clients/CMakeLists.txt` inside the existing `if(BUILD_TESTING)` block, before `ament_lint_auto_find_test_dependencies()`:
```cmake
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_recovery_node test/test_recovery_node.cpp)
  target_link_libraries(test_recovery_node recovery_node)
  ament_target_dependencies(test_recovery_node ${dependencies})
```

`ros_env/src/bizon_behavior_clients/test/test_recovery_node.cpp`:
```cpp
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "bizon_behavior_clients/plugins/control/recovery_node.hpp"

using bizon_behavior_clients::RecoveryNode;

namespace {

/// Records every tick so the test can assert on ordering, and returns a
/// scripted sequence of results.
class ScriptedAction : public BT::SyncActionNode
{
public:
  ScriptedAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    trace->push_back(name());
    if (call_index_ < results.size()) {
      return results[call_index_++];
    }
    return results.empty() ? BT::NodeStatus::SUCCESS : results.back();
  }

  std::vector<BT::NodeStatus> results;
  std::vector<std::string> * trace{nullptr};

private:
  size_t call_index_{0};
};

struct Fixture
{
  BT::BehaviorTreeFactory factory;
  std::vector<std::string> trace;
  ScriptedAction * work{nullptr};
  ScriptedAction * recovery{nullptr};
  BT::Tree tree;

  void build(const std::vector<BT::NodeStatus> & work_results,
             const std::vector<BT::NodeStatus> & recovery_results,
             int retries)
  {
    factory.registerNodeType<RecoveryNode>("RecoveryNode");
    factory.registerNodeType<ScriptedAction>("Work");
    factory.registerNodeType<ScriptedAction>("Recovery");

    const std::string xml =
      R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">)"
      R"(<RecoveryNode number_of_retries=")" + std::to_string(retries) + R"(">)"
      R"(<Work name="work"/><Recovery name="recovery"/>)"
      R"(</RecoveryNode></BehaviorTree></root>)";

    tree = factory.createTreeFromText(xml);
    for (auto & subtree : tree.subtrees) {
      for (auto & node : subtree->nodes) {
        if (auto * a = dynamic_cast<ScriptedAction *>(node.get())) {
          a->trace = &trace;
          if (a->name() == "work") { a->results = work_results; work = a; }
          else { a->results = recovery_results; recovery = a; }
        }
      }
    }
  }
};

}  // namespace

TEST(RecoveryNode, ReturnsSuccessWithoutRunningRecovery)
{
  Fixture f;
  f.build({BT::NodeStatus::SUCCESS}, {BT::NodeStatus::SUCCESS}, 3);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.trace, (std::vector<std::string>{"work"}));
}

// The core contract: the work branch is never re-entered until recovery ran.
TEST(RecoveryNode, RunsRecoveryBetweenWorkAttempts)
{
  Fixture f;
  f.build({BT::NodeStatus::FAILURE, BT::NodeStatus::SUCCESS}, {BT::NodeStatus::SUCCESS}, 3);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.trace, (std::vector<std::string>{"work", "recovery", "work"}));
}

TEST(RecoveryNode, GivesUpAfterRetriesExhausted)
{
  Fixture f;
  f.build({BT::NodeStatus::FAILURE}, {BT::NodeStatus::SUCCESS}, 2);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.trace,
    (std::vector<std::string>{"work", "recovery", "work", "recovery", "work"}));
}

// A failed recovery must stop the tree, not loop forever.
TEST(RecoveryNode, FailsImmediatelyWhenRecoveryFails)
{
  Fixture f;
  f.build({BT::NodeStatus::FAILURE}, {BT::NodeStatus::FAILURE}, 3);
  EXPECT_EQ(f.tree.tickWhileRunning(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.trace, (std::vector<std::string>{"work", "recovery"}));
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ros_env
colcon build --packages-select bizon_behavior_clients
```
Expected: FAIL — `plugins/control/recovery_node.hpp: No such file or directory`.

- [ ] **Step 3: Write the header**

`ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/control/recovery_node.hpp`:
```cpp
#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RECOVERY_NODE_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"

namespace bizon_behavior_clients
{
/// Two-child control node. Child 0 does the work, child 1 recovers from a
/// failure of child 0. The work branch is never retried until the recovery
/// branch has completed successfully.
class RecoveryNode : public BT::ControlNode
{
public:
  RecoveryNode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 3, "Recovery attempts before giving up")};
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  int retry_count_{0};
  unsigned current_child_idx_{0};
};
}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RECOVERY_NODE_HPP_
```

- [ ] **Step 4: Write the implementation**

`ros_env/src/bizon_behavior_clients/plugins/control/recovery_node.cpp`:
```cpp
#include "bizon_behavior_clients/plugins/control/recovery_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bizon_behavior_clients
{

RecoveryNode::RecoveryNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus RecoveryNode::tick()
{
  if (children_nodes_.size() != 2) {
    // BT::RuntimeError is what the rest of this package throws (see foreach_node.cpp:20).
    throw BT::RuntimeError("RecoveryNode '" + name() + "' must have exactly 2 children");
  }

  int max_retries = 3;
  getInput("number_of_retries", max_retries);

  setStatus(BT::NodeStatus::RUNNING);

  while (true) {
    if (current_child_idx_ == 0) {
      const BT::NodeStatus child_status = children_nodes_[0]->executeTick();

      if (child_status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }

      if (child_status == BT::NodeStatus::SUCCESS) {
        halt();
        return BT::NodeStatus::SUCCESS;
      }

      // FAILURE
      haltChild(0);
      if (retry_count_ >= max_retries) {
        RCLCPP_ERROR(
          rclcpp::get_logger("RecoveryNode"),
          "[%s] work branch failed and %d retries are exhausted, giving up",
          name().c_str(), max_retries);
        halt();
        return BT::NodeStatus::FAILURE;
      }

      RCLCPP_WARN(
        rclcpp::get_logger("RecoveryNode"),
        "[%s] work branch failed, running recovery (attempt %d/%d)",
        name().c_str(), retry_count_ + 1, max_retries);
      current_child_idx_ = 1;
      continue;
    }

    // current_child_idx_ == 1: recovery branch
    const BT::NodeStatus recovery_status = children_nodes_[1]->executeTick();

    if (recovery_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }

    haltChild(1);

    if (recovery_status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RecoveryNode"),
        "[%s] recovery branch itself failed, giving up", name().c_str());
      halt();
      return BT::NodeStatus::FAILURE;
    }

    // Recovery succeeded: count the attempt and let the work branch run again.
    retry_count_++;
    current_child_idx_ = 0;
  }
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bizon_behavior_clients::RecoveryNode>("RecoveryNode");
}
```

- [ ] **Step 5: Wire it into the build**

In `ros_env/src/bizon_behavior_clients/CMakeLists.txt`, after the `foreach_node` block, add:
```cmake
add_library(recovery_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs recovery_node)
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
cd ros_env
colcon build --packages-select bizon_behavior_clients && colcon test --packages-select bizon_behavior_clients && colcon test-result --verbose
```
Expected: 4 tests, all PASS. In particular `RunsRecoveryBetweenWorkAttempts` proves the piece-dragging path is closed.

- [ ] **Step 7: Write the new tree**

`ros_env/src/bizon_behavior_clients/behavior_trees/chess_game.xml`:
```xml
<root BTCPP_format="4">
    <!-- Main game loop. The work branch performs one full move; the recovery
         branch always releases the piece and lifts clear before the work
         branch is allowed to run again. -->
    <BehaviorTree ID="MainTree">
        <RecoveryNode number_of_retries="3">

            <Sequence name="PlayOneMove">
                <Script code="home_position := '-1.5807;0.0;1.5807;0.0'; hand_open_position := '0.20;0.20;0.20'; hand_close_position := '0.04;0.04;0.04'" />
                <ArmActionClient player_side="{player_side}"
                    target_joint_positions="{home_position}"
                    target_hand_position="{hand_close_position}" />
                <BoardActionIsaacClient server_name="board_action" sec="15" nanosec="0"
                    player_side="{player_side}" fen="{fen}" />
                <MakeDecisionClient player_side="{player_side}" fen="{fen}"
                    hand_open_position="{hand_open_position}"
                    hand_close_position="{hand_close_position}"
                    move_type="{move_type}"
                    move_count="{move_count}"
                    move_from1="{move_from1}" move_from_down1="{move_from_down1}"
                    move_to1="{move_to1}" move_to_down1="{move_to_down1}"
                    move_from2="{move_from2}" move_from_down2="{move_from_down2}"
                    move_to2="{move_to2}" move_to_down2="{move_to_down2}"
                    move_from3="{move_from3}" move_from_down3="{move_from_down3}"
                    move_to3="{move_to3}" move_to_down3="{move_to_down3}" />
                <Foreach name="MoveLoop"
                    move_count="{move_count}"
                    move_from1="{move_from1}" move_from_down1="{move_from_down1}"
                    move_to1="{move_to1}" move_to_down1="{move_to_down1}"
                    move_from2="{move_from2}" move_from_down2="{move_from_down2}"
                    move_to2="{move_to2}" move_to_down2="{move_to_down2}"
                    move_from3="{move_from3}" move_from_down3="{move_from_down3}"
                    move_to3="{move_to3}" move_to_down3="{move_to_down3}"
                    move_from="{move_from}" move_from_down="{move_from_down}"
                    move_to="{move_to}" move_to_down="{move_to_down}">
                    <SubTree ID="MoveSequence" _autoremap="true"/>
                </Foreach>
                <ArmActionClient player_side="{player_side}"
                    target_joint_positions="{home_position}"
                    target_hand_position="{hand_close_position}" />
                <ConditionNode name="CheckGameOver" param1="{move_type}" param2="killking" />
            </Sequence>

            <SubTree ID="RecoverArm" _autoremap="true"/>

        </RecoveryNode>
    </BehaviorTree>

    <!-- Order matters and is the whole point: open the gripper BEFORE moving,
         so a piece held at the moment of failure is set down where it is
         rather than dragged across the board. -->
    <BehaviorTree ID="RecoverArm">
        <Sequence name="RecoverArm">
            <ArmActionClient player_side="{player_side}"
                target_joint_positions="{recovery_release_position}"
                target_hand_position="{hand_open_position}" />
            <ArmActionClient player_side="{player_side}"
                target_joint_positions="{recovery_lift_position}"
                target_hand_position="{hand_open_position}" />
            <ArmActionClient player_side="{player_side}"
                target_joint_positions="{home_position}"
                target_hand_position="{hand_close_position}" />
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="MoveSequence">
        <Sequence name="MoveSequence">
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_from}"
                target_hand_position="{hand_open_position}" />
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_from_down}"
                target_hand_position="{hand_open_position}" />
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_from_down}"
                target_hand_position="{hand_close_position}" />
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_from}"
                target_hand_position="{hand_close_position}" />
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_to}"
                target_hand_position="{hand_close_position}" />
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_to_down}"
                target_hand_position="{hand_close_position}" />
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_to_down}"
                target_hand_position="{hand_open_position}" />
            <ArmActionClient player_side="{player_side}" target_joint_positions="{move_to}"
                target_hand_position="{hand_open_position}" />
        </Sequence>
    </BehaviorTree>
</root>
```

- [ ] **Step 8: Seed the recovery blackboard entries and load the new tree**

In `ros_env/src/bizon_behavior_clients/src/main.cpp`, add `"recovery_node"` to `plugin_lib_names_` (line 30-38 list).

Change line 44 to load the new tree:
```cpp
    fs::path bt_file = fs::path(ament_index_cpp::get_package_share_directory("bizon_behavior_clients")) / "behavior_trees" / "chess_game.xml";
```

After the existing `blackboard->set<std::string>("player_side", player_side);` (line 82), add:
```cpp
    // Recovery waypoints. The release pose is the current-height retreat used
    // to set a held piece down; the lift pose clears the board before homing.
    // Values match the joint layout used throughout the trees:
    // {rev1, pris1, rev2, rev3}, with pris1 = 0.0 meaning fully retracted.
    blackboard->set<std::string>("recovery_release_position", "-1.5807;0.155;1.5807;0.0");
    blackboard->set<std::string>("recovery_lift_position", "-1.5807;0.0;1.5807;0.0");
```

Also change the tick loop at line 111-116 so the game actually loops instead of running once:
```cpp
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
        status = tree.tickWhileRunning();
    }
    std::cout << "Tree finished with: " << status << std::endl;
```

- [ ] **Step 9: Verify recovery fires in simulation**

Build and launch as in Task 2 Step 6, but with the tree now `chess_game.xml`. To force a failure, temporarily set an unreachable target: in Isaac Sim, pause the physics mid-`MoveSequence` so MoveIt reports a failure.

Expected in the log, in this order:
```
[RecoveryNode] work branch failed, running recovery (attempt 1/3)
... ArmActionClient ... hand_open_position ...
... ArmActionClient ... recovery_lift_position ...
... ArmActionClient ... home_position ...
```
Expected on screen: the gripper opens **before** the arm translates. No piece is dragged.

- [ ] **Step 10: Commit**

```bash
git add ros_env/src/bizon_behavior_clients
git commit -m "feat(bt): add RecoveryNode with release-before-retreat recovery subtree"
```

---

### Task 4: Arm action server

Closes F3. Moves MoveIt into a lifecycle-managed plugin so it can be deactivated on E-stop and cancelled mid-motion.

**Files:**
- Create: `ros_env/src/bizon_msgs/action/Arm.action`
- Modify: `ros_env/src/bizon_msgs/CMakeLists.txt` (add to `rosidl_generate_interfaces`)
- Create: `ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/plugins/arm_plugin.hpp`
- Create: `ros_env/src/bizon_behavior_servers/plugins/arm_plugin.cpp`
- Modify: `ros_env/src/bizon_behavior_servers/behavior_plugin.xml`
- Modify: `ros_env/src/bizon_behavior_servers/CMakeLists.txt`
- Modify: `ros_env/src/bizon_player_bringup/params/bizon_behavior_params.yaml` and `bizon_behavior_params_bizon3.yaml`
- Rewrite: `ros_env/src/bizon_behavior_clients/plugins/action/arm_action_client_node.cpp` and its header

**Interfaces:**
- Consumes: `RecoveryNode` and the tree from Task 3 — the XML must not need editing, so the new `ArmActionClient` keeps ports `player_side`, `target_joint_positions`, `target_hand_position` and adds an optional `server_name` (default `arm_action`) inherited from `providedBasicPorts`.
- Produces: action `bizon_msgs/action/Arm` and behavior plugin `bizon_behaviors/ArmPlugin` registered under the behavior id `arm_action`. Task 7 extends this plugin with planning-scene updates.

- [ ] **Step 1: Define the action**

`ros_env/src/bizon_msgs/action/Arm.action`:
```
#goal definition
float64[] target_joint_positions
float64[] target_hand_position
string player_side
---
#result definition
uint16 error_code
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition
string phase
```

In `ros_env/src/bizon_msgs/CMakeLists.txt`, add `"action/Arm.action"` to the `rosidl_generate_interfaces` list.

- [ ] **Step 2: Build the interface and verify it exists**

```bash
cd ros_env
colcon build --packages-select bizon_msgs && . install/setup.bash && ros2 interface show bizon_msgs/action/Arm
```
Expected: the three sections print without error.

- [ ] **Step 3: Write the plugin header**

`ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/plugins/arm_plugin.hpp`:
```cpp
#ifndef BIZON_BEHAVIOR_SERVERS__PLUGINS__ARM_PLUGIN_HPP_
#define BIZON_BEHAVIOR_SERVERS__PLUGINS__ARM_PLUGIN_HPP_

#include <future>
#include <memory>
#include <string>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>

#include "bizon_behavior_servers/timed_behavior.hpp"
#include "bizon_msgs/action/arm.hpp"

namespace bizon_behaviors
{
/// Owns the MoveGroupInterface for one robot. Arm motion runs to completion
/// before the gripper is commanded, so a settling correction can never occur
/// while the fingers are closing.
class ArmPlugin : public TimedBehavior<bizon_msgs::action::Arm>
{
public:
  using ArmAction = bizon_msgs::action::Arm;

  ArmPlugin();
  ~ArmPlugin() override;

  void onConfigure() override;
  void onCleanup() override;

  ResultStatus onRun(const std::shared_ptr<const ArmAction::Goal> command) override;
  ResultStatus onCycleUpdate() override;

private:
  enum class Phase
  {
    ARM_MOVING,
    HAND_MOVING,
  };

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_hand_;

  std::future<moveit::core::MoveItErrorCode> future_arm_;
  std::future<moveit::core::MoveItErrorCode> future_hand_;

  Phase phase_{Phase::ARM_MOVING};
  std::vector<double> target_hand_position_;
  double planning_time_{15.0};
};
}  // namespace bizon_behaviors

#endif  // BIZON_BEHAVIOR_SERVERS__PLUGINS__ARM_PLUGIN_HPP_
```

- [ ] **Step 4: Write the plugin implementation**

`ros_env/src/bizon_behavior_servers/plugins/arm_plugin.cpp`:
```cpp
#include "bizon_behavior_servers/plugins/arm_plugin.hpp"

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace bizon_behaviors
{

ArmPlugin::ArmPlugin() = default;
ArmPlugin::~ArmPlugin() = default;

void ArmPlugin::onConfigure()
{
  auto node = node_.lock();

  if (!node->has_parameter(behavior_name_ + ".planning_time")) {
    node->declare_parameter(behavior_name_ + ".planning_time", 15.0);
  }
  node->get_parameter(behavior_name_ + ".planning_time", planning_time_);

  moveit::planning_interface::MoveGroupInterface::Options arm_options(
    "arm_group", "robot_description", node->get_namespace());
  moveit::planning_interface::MoveGroupInterface::Options hand_options(
    "hand_group", "robot_description", node->get_namespace());

  move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, arm_options);
  move_group_hand_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, hand_options);

  move_group_arm_->setPlanningTime(planning_time_);
  move_group_hand_->setPlanningTime(planning_time_);

  RCLCPP_INFO(node->get_logger(), "[%s] ArmPlugin configured", behavior_name_.c_str());
}

void ArmPlugin::onCleanup()
{
  move_group_arm_.reset();
  move_group_hand_.reset();
}

ResultStatus ArmPlugin::onRun(const std::shared_ptr<const ArmAction::Goal> command)
{
  auto node = node_.lock();

  if (command->target_joint_positions.empty() || command->target_hand_position.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] empty joint target in goal", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 1};
  }

  if (move_group_arm_->getCurrentJointValues().empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] no joint state available", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 2};
  }

  target_hand_position_ = command->target_hand_position;

  move_group_arm_->setJointValueTarget(command->target_joint_positions);
  future_arm_ = std::async(std::launch::async, [this]() { return move_group_arm_->move(); });
  phase_ = Phase::ARM_MOVING;

  return ResultStatus{Status::SUCCEEDED, 0};
}

ResultStatus ArmPlugin::onCycleUpdate()
{
  auto node = node_.lock();

  if (phase_ == Phase::ARM_MOVING) {
    if (future_arm_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      return ResultStatus{Status::RUNNING, 0};
    }
    const auto result = future_arm_.get();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "[%s] arm move failed: %d", behavior_name_.c_str(), result.val);
      return ResultStatus{Status::FAILED, static_cast<uint16_t>(-result.val)};
    }

    move_group_hand_->setJointValueTarget(target_hand_position_);
    future_hand_ = std::async(std::launch::async, [this]() { return move_group_hand_->move(); });
    phase_ = Phase::HAND_MOVING;
    return ResultStatus{Status::RUNNING, 0};
  }

  if (future_hand_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    return ResultStatus{Status::RUNNING, 0};
  }
  const auto result = future_hand_.get();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "[%s] hand move failed: %d", behavior_name_.c_str(), result.val);
    return ResultStatus{Status::FAILED, static_cast<uint16_t>(-result.val)};
  }

  return ResultStatus{Status::SUCCEEDED, 0};
}

}  // namespace bizon_behaviors

PLUGINLIB_EXPORT_CLASS(bizon_behaviors::ArmPlugin, bizon_core::Behavior)
```

- [ ] **Step 5: Register the plugin and wire the build**

Add to `ros_env/src/bizon_behavior_servers/behavior_plugin.xml`, inside `<class_libraries>`:
```xml
	<library path="bizon_arm_behavior">
	  <class name="bizon_behaviors/ArmPlugin" type="bizon_behaviors::ArmPlugin" base_class_type="bizon_core::Behavior">
	    <description>MoveIt-backed arm and gripper motion</description>
	  </class>
	</library>
```

In `ros_env/src/bizon_behavior_servers/CMakeLists.txt`, add `moveit_ros_planning_interface` and `bizon_msgs` to the `find_package` calls and to the `dependencies` list, then after the `bizon_wait_behavior` block add:
```cmake
add_library(bizon_arm_behavior SHARED plugins/arm_plugin.cpp)
ament_target_dependencies(bizon_arm_behavior ${dependencies})
```
Add `bizon_arm_behavior` to both the `install(TARGETS ...)` list and the `ament_export_libraries(...)` list.

In `ros_env/src/bizon_behavior_servers/package.xml`, add:
```xml
  <build_depend>moveit_ros_planning_interface</build_depend>
  <exec_depend>moveit_ros_planning_interface</exec_depend>
```

- [ ] **Step 6: Enable the behavior in both param files**

In `bizon_player_bringup/params/bizon_behavior_params.yaml` and `bizon_behavior_params_bizon3.yaml`, change the plugin list and add the entry:
```yaml
    behavior_plugins: ["wait_action", "board_action", "arm_action"]
    wait_action:
      plugin: "bizon_behaviors/WaitPlugin"
    board_action:
      plugin: "bizon_behaviors/BoardPlugin"
    arm_action:
      plugin: "bizon_behaviors/ArmPlugin"
      planning_time: 15.0
```

- [ ] **Step 7: Rewrite the BT client node as an action client**

Replace `ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/action/arm_action_client_node.hpp` entirely:
```cpp
#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_

#include <string>
#include <vector>

#include "bizon_behavior_clients/bt_action_client_node.hpp"
#include "bizon_msgs/action/arm.hpp"

namespace bizon_behavior_clients
{
class ArmActionClientNode : public BtActionClientNode<bizon_msgs::action::Arm>
{
public:
  ArmActionClientNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus onResultReceived(
    const rclcpp_action::ClientGoalHandle<bizon_msgs::action::Arm>::WrappedResult & result) override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("player_side", "white or black"),
      BT::InputPort<std::vector<double>>("target_joint_positions", "Arm joint targets"),
      BT::InputPort<std::vector<double>>("target_hand_position", "Gripper finger targets"),
    });
  }
};
}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__ARM_ACTION_CLIENT_NODE_HPP_
```

Replace `plugins/action/arm_action_client_node.cpp` entirely:
```cpp
#include "bizon_behavior_clients/plugins/action/arm_action_client_node.hpp"

namespace bizon_behavior_clients
{

ArmActionClientNode::ArmActionClientNode(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionClientNode<bizon_msgs::action::Arm>(xml_tag_name, action_name, conf)
{
}

void ArmActionClientNode::on_tick()
{
  std::vector<double> joints;
  std::vector<double> hand;
  std::string player_side;

  getInput("target_joint_positions", joints);
  getInput("target_hand_position", hand);
  getInput("player_side", player_side);

  goal_.target_joint_positions = joints;
  goal_.target_hand_position = hand;
  goal_.player_side = player_side;
}

BT::NodeStatus ArmActionClientNode::onResultReceived(
  const rclcpp_action::ClientGoalHandle<bizon_msgs::action::Arm>::WrappedResult & result)
{
  if (result.result->error_code != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ArmActionClientNode"),
      "Arm action reported error_code %u", result.result->error_code);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ArmActionClientNode::on_aborted()
{
  RCLCPP_ERROR(rclcpp::get_logger("ArmActionClientNode"), "Arm action aborted");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ArmActionClientNode::on_cancelled()
{
  RCLCPP_WARN(rclcpp::get_logger("ArmActionClientNode"), "Arm action cancelled");
  return BT::NodeStatus::FAILURE;
}

}  // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<bizon_behavior_clients::ArmActionClientNode>(
      name, "arm_action", config);
  };
  factory.registerBuilder<bizon_behavior_clients::ArmActionClientNode>("ArmActionClient", builder);
}
```

Add `bizon_msgs` to `find_package` and `dependencies` in `bizon_behavior_clients/CMakeLists.txt` if not already present, and to `package.xml`.

- [ ] **Step 8: Build everything**

```bash
cd ros_env
colcon build --packages-up-to bizon_behavior_clients bizon_behavior_servers && . install/setup.bash
```
Expected: clean build.

- [ ] **Step 9: Verify the action server appears and the game still plays**

Launch as in Task 2 Step 6, then in another container shell:
```bash
ros2 action list | grep arm_action
```
Expected: `/bizon2/arm_action`.

Expected behavior: one full white move executes exactly as before. The difference is that `ros2 lifecycle set /bizon2/behavior_server deactivate` now stops the arm accepting goals.

- [ ] **Step 10: Commit**

```bash
git add ros_env/src/bizon_msgs ros_env/src/bizon_behavior_servers ros_env/src/bizon_behavior_clients ros_env/src/bizon_player_bringup
git commit -m "refactor(arm): move MoveIt behind a lifecycle-managed arm_action server"
```

---

### Task 5: Decision action server with a timed Stockfish subprocess

Closes F4 and completes F7. The `fork()` leaves the BT process and gains a timeout.

**Files:**
- Create: `ros_env/src/bizon_msgs/action/Decision.action`
- Modify: `ros_env/src/bizon_msgs/CMakeLists.txt`
- Create: `ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/stockfish_process.hpp`
- Create: `ros_env/src/bizon_behavior_servers/src/stockfish_process.cpp`
- Create: `ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/plugins/decision_plugin.hpp`
- Create: `ros_env/src/bizon_behavior_servers/plugins/decision_plugin.cpp`
- Create: `ros_env/src/bizon_behavior_clients/plugins/action/decision_action_client_node.cpp` and its header
- Test: `ros_env/src/bizon_behavior_servers/test/test_stockfish_process.cpp`
- Delete at the end: `plugins/action/make_decision_client_node.cpp`, its header, and their `CMakeLists.txt` entries

**Interfaces:**
- Consumes: `bizon_chess::RobotParams`, `squareToWorld`, `worldToJointAngles` from Task 1.
- Produces: action `bizon_msgs/action/Decision`, behavior id `decision_action`, and BT node `MakeDecisionClient` — **the XML tag name is unchanged** so `chess_game.xml` from Task 3 needs no edit.

- [ ] **Step 1: Define the action**

`ros_env/src/bizon_msgs/action/Decision.action`:
```
#goal definition
string fen
string player_side
---
#result definition
string move_type
int32 move_count
float64[] move_from1
float64[] move_from_down1
float64[] move_to1
float64[] move_to_down1
float64[] move_from2
float64[] move_from_down2
float64[] move_to2
float64[] move_to_down2
float64[] move_from3
float64[] move_from_down3
float64[] move_to3
float64[] move_to_down3
uint16 error_code
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition
string stage
```

Add `"action/Decision.action"` to `bizon_msgs/CMakeLists.txt`.

- [ ] **Step 2: Write the failing test for the subprocess wrapper**

`ros_env/src/bizon_behavior_servers/test/test_stockfish_process.cpp`:
```cpp
#include <gtest/gtest.h>
#include <chrono>
#include <string>

#include "bizon_behavior_servers/stockfish_process.hpp"

using bizon_behaviors::StockfishProcess;

// Requires the `stockfish` binary on PATH, as the README's install step provides.
TEST(StockfishProcess, StartsAndHandshakes)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();
  EXPECT_TRUE(sf.isRunning());
  sf.stop();
  EXPECT_FALSE(sf.isRunning());
}

TEST(StockfishProcess, ReturnsBestMoveForOpeningPosition)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();

  std::string best;
  ASSERT_TRUE(sf.bestMove(
    "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    10, std::chrono::seconds(10), best)) << sf.lastError();

  // A legal long-algebraic move: two squares, optionally a promotion char.
  ASSERT_GE(best.size(), 4u);
  EXPECT_GE(best[0], 'a'); EXPECT_LE(best[0], 'h');
  EXPECT_GE(best[1], '1'); EXPECT_LE(best[1], '8');
  EXPECT_GE(best[2], 'a'); EXPECT_LE(best[2], 'h');
  EXPECT_GE(best[3], '1'); EXPECT_LE(best[3], '8');
  sf.stop();
}

// The bug this class exists to fix: a hung engine must not block forever.
TEST(StockfishProcess, TimesOutInsteadOfBlockingForever)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();

  std::string best;
  const auto t0 = std::chrono::steady_clock::now();
  // A zero-length deadline can never be met, so this must return false fast.
  const bool ok = sf.bestMove(
    "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    30, std::chrono::milliseconds(1), best);
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_FALSE(ok);
  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 3);
  sf.stop();
}

TEST(StockfishProcess, ReportsFailureForMissingBinary)
{
  StockfishProcess sf("definitely-not-a-real-engine-binary");
  EXPECT_FALSE(sf.start(std::chrono::seconds(2)));
  EXPECT_FALSE(sf.lastError().empty());
}
```

Add to the `if(BUILD_TESTING)` block of `bizon_behavior_servers/CMakeLists.txt`:
```cmake
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stockfish_process test/test_stockfish_process.cpp src/stockfish_process.cpp)
  target_include_directories(test_stockfish_process PRIVATE include)
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
cd ros_env
colcon build --packages-select bizon_behavior_servers
```
Expected: FAIL — `stockfish_process.hpp: No such file or directory`.

- [ ] **Step 4: Write the header**

`ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/stockfish_process.hpp`:
```cpp
#ifndef BIZON_BEHAVIOR_SERVERS__STOCKFISH_PROCESS_HPP_
#define BIZON_BEHAVIOR_SERVERS__STOCKFISH_PROCESS_HPP_

#include <chrono>
#include <string>
#include <sys/types.h>

namespace bizon_behaviors
{
/// A UCI engine subprocess with deadlines on every read.
///
/// Replaces the fork()+blocking-read that lived in the BT node constructor.
/// Every read is bounded by poll(), so a wedged engine surfaces as a failed
/// action instead of freezing the behavior tree with the arm mid-air.
class StockfishProcess
{
public:
  explicit StockfishProcess(std::string binary = "stockfish");
  ~StockfishProcess();

  StockfishProcess(const StockfishProcess &) = delete;
  StockfishProcess & operator=(const StockfishProcess &) = delete;

  /// Spawn the engine and complete the uci/isready handshake.
  bool start(std::chrono::milliseconds timeout);

  /// Terminate the engine and reap it. Safe to call when not running.
  void stop();

  bool isRunning() const { return pid_ > 0; }

  /// Search `fen` to `depth` and return the bestmove token.
  /// Returns false on timeout, engine death, or a malformed reply.
  bool bestMove(
    const std::string & fen,
    int depth,
    std::chrono::milliseconds timeout,
    std::string & best_move_out);

  /// Configure engine resource use. Call after start(). On Jetson keep
  /// threads at 1 and hash small so the engine does not contend with TensorRT.
  bool setOption(const std::string & name, const std::string & value);

  const std::string & lastError() const { return last_error_; }

private:
  bool writeLine(const std::string & line);
  /// Read until `token` appears at the start of a line, or the deadline passes.
  bool readUntil(
    const std::string & token,
    std::chrono::steady_clock::time_point deadline,
    std::string & line_out);

  std::string binary_;
  std::string last_error_;
  std::string pending_;
  int to_engine_{-1};
  int from_engine_{-1};
  pid_t pid_{-1};
};
}  // namespace bizon_behaviors

#endif  // BIZON_BEHAVIOR_SERVERS__STOCKFISH_PROCESS_HPP_
```

- [ ] **Step 5: Write the implementation**

`ros_env/src/bizon_behavior_servers/src/stockfish_process.cpp`:
```cpp
#include "bizon_behavior_servers/stockfish_process.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace bizon_behaviors
{

StockfishProcess::StockfishProcess(std::string binary)
: binary_(std::move(binary))
{
}

StockfishProcess::~StockfishProcess()
{
  stop();
}

bool StockfishProcess::start(std::chrono::milliseconds timeout)
{
  if (isRunning()) {
    return true;
  }

  int to_pipe[2];
  int from_pipe[2];
  if (pipe(to_pipe) != 0 || pipe(from_pipe) != 0) {
    last_error_ = std::string("pipe() failed: ") + std::strerror(errno);
    return false;
  }

  const pid_t pid = fork();
  if (pid < 0) {
    last_error_ = std::string("fork() failed: ") + std::strerror(errno);
    return false;
  }

  if (pid == 0) {
    // Child: only async-signal-safe calls until execlp.
    ::close(to_pipe[1]);
    ::close(from_pipe[0]);
    ::dup2(to_pipe[0], STDIN_FILENO);
    ::dup2(from_pipe[1], STDOUT_FILENO);
    const int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      ::dup2(devnull, STDERR_FILENO);
      ::close(devnull);
    }
    ::close(to_pipe[0]);
    ::close(from_pipe[1]);
    ::execlp(binary_.c_str(), binary_.c_str(), static_cast<char *>(nullptr));
    ::_exit(127);
  }

  ::close(to_pipe[0]);
  ::close(from_pipe[1]);
  to_engine_ = to_pipe[1];
  from_engine_ = from_pipe[0];
  pid_ = pid;
  pending_.clear();

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string line;
  if (!writeLine("uci") || !readUntil("uciok", deadline, line)) {
    last_error_ = "engine did not answer uci handshake (" + last_error_ + ")";
    stop();
    return false;
  }
  if (!writeLine("isready") || !readUntil("readyok", deadline, line)) {
    last_error_ = "engine did not answer isready (" + last_error_ + ")";
    stop();
    return false;
  }

  last_error_.clear();
  return true;
}

void StockfishProcess::stop()
{
  if (to_engine_ >= 0) {
    writeLine("quit");
    ::close(to_engine_);
    to_engine_ = -1;
  }
  if (from_engine_ >= 0) {
    ::close(from_engine_);
    from_engine_ = -1;
  }
  if (pid_ > 0) {
    int status = 0;
    // Give the engine a moment to exit on `quit`, then insist.
    for (int i = 0; i < 20; ++i) {
      const pid_t r = ::waitpid(pid_, &status, WNOHANG);
      if (r == pid_ || r < 0) {
        pid_ = -1;
        return;
      }
      ::usleep(10000);
    }
    ::kill(pid_, SIGKILL);
    ::waitpid(pid_, &status, 0);
    pid_ = -1;
  }
}

bool StockfishProcess::setOption(const std::string & name, const std::string & value)
{
  return writeLine("setoption name " + name + " value " + value);
}

bool StockfishProcess::bestMove(
  const std::string & fen,
  int depth,
  std::chrono::milliseconds timeout,
  std::string & best_move_out)
{
  if (!isRunning()) {
    last_error_ = "engine is not running";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  if (!writeLine("position fen " + fen) ||
      !writeLine("go depth " + std::to_string(depth)))
  {
    return false;
  }

  std::string line;
  if (!readUntil("bestmove", deadline, line)) {
    // Stop the search so the engine is reusable for the next goal.
    writeLine("stop");
    return false;
  }

  const size_t sp = line.find(' ');
  if (sp == std::string::npos || sp + 1 >= line.size()) {
    last_error_ = "malformed bestmove line: " + line;
    return false;
  }
  size_t end = line.find(' ', sp + 1);
  if (end == std::string::npos) {
    end = line.size();
  }
  best_move_out = line.substr(sp + 1, end - sp - 1);

  if (best_move_out.empty() || best_move_out == "(none)") {
    last_error_ = "engine reported no legal move";
    return false;
  }
  return true;
}

bool StockfishProcess::writeLine(const std::string & line)
{
  if (to_engine_ < 0) {
    last_error_ = "engine stdin is closed";
    return false;
  }
  const std::string payload = line + "\n";
  size_t written = 0;
  while (written < payload.size()) {
    const ssize_t n = ::write(to_engine_, payload.data() + written, payload.size() - written);
    if (n < 0) {
      if (errno == EINTR) { continue; }
      last_error_ = std::string("write() failed: ") + std::strerror(errno);
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

bool StockfishProcess::readUntil(
  const std::string & token,
  std::chrono::steady_clock::time_point deadline,
  std::string & line_out)
{
  char buffer[4096];

  while (true) {
    // Serve any complete line already buffered before touching the pipe.
    size_t nl = pending_.find('\n');
    while (nl != std::string::npos) {
      std::string line = pending_.substr(0, nl);
      pending_.erase(0, nl + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (line.rfind(token, 0) == 0) {
        line_out = line;
        return true;
      }
      nl = pending_.find('\n');
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      last_error_ = "timed out waiting for '" + token + "'";
      return false;
    }

    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();

    struct pollfd pfd;
    pfd.fd = from_engine_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    const int pr = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (pr < 0) {
      if (errno == EINTR) { continue; }
      last_error_ = std::string("poll() failed: ") + std::strerror(errno);
      return false;
    }
    if (pr == 0) {
      last_error_ = "timed out waiting for '" + token + "'";
      return false;
    }

    const ssize_t n = ::read(from_engine_, buffer, sizeof(buffer));
    if (n < 0) {
      if (errno == EINTR) { continue; }
      last_error_ = std::string("read() failed: ") + std::strerror(errno);
      return false;
    }
    if (n == 0) {
      last_error_ = "engine closed its output pipe";
      return false;
    }
    pending_.append(buffer, static_cast<size_t>(n));
  }
}

}  // namespace bizon_behaviors
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
cd ros_env
colcon build --packages-select bizon_behavior_servers && colcon test --packages-select bizon_behavior_servers --ctest-args -R test_stockfish_process && colcon test-result --verbose
```
Expected: 4 tests PASS. `TimesOutInsteadOfBlockingForever` is the one that proves F4 is closed.

- [ ] **Step 7: Port the decision logic into `DecisionPlugin`**

Move the private methods of `MakeDecisionNode` (`make_decision_client_node.hpp:50-60`: `get_last_move_from_text`, `possible_next_moves_from_valid_fen_`, `apply_move_to_fen`, `who_is_owner_of_move`, `get_move_type`, `get_best_move`, `write_to_text_file`, `is_checkmate`, `reset_values`) into `bizon_behaviors::DecisionPlugin`, with these substitutions:

- Every `send_command`/`read_until` pair becomes a `StockfishProcess` call.
- `get_box_location_wrt_world` becomes `bizon_chess::squareToWorld`.
- `calculate_joint_angles` becomes `bizon_chess::worldToJointAngles`, and a `false` return now aborts the goal with `error_code = 3` instead of producing NaN targets.
- The constants at `make_decision_client_node.hpp:103-110` become a `bizon_chess::RobotParams` filled from ROS parameters declared in `onConfigure()` under the `decision_action.` prefix.
- `last_moves_path` becomes a ROS parameter `decision_action.state_file` defaulting to `/tmp/bizon_<player_side>_last_moves.txt`, replacing the hardcoded `/home/user/Documents/bizon_chess_player`.

`onRun` starts the engine if needed and validates the goal; `onCycleUpdate` performs one search and fills the result. Follow the `ArmPlugin` structure from Task 4 Step 4 exactly.

Register in `behavior_plugin.xml`:
```xml
	<library path="bizon_decision_behavior">
	  <class name="bizon_behaviors/DecisionPlugin" type="bizon_behaviors::DecisionPlugin" base_class_type="bizon_core::Behavior">
	    <description>Stockfish-backed move decision</description>
	  </class>
	</library>
```

Add to both param files:
```yaml
    behavior_plugins: ["wait_action", "board_action", "arm_action", "decision_action"]
    decision_action:
      plugin: "bizon_behaviors/DecisionPlugin"
      search_depth: 10
      search_timeout: 10.0
      engine_threads: 1
      engine_hash_mb: 16
      box_size: 0.03718857142
      robot_base_offset_x: -0.3
      link_l1: 0.29
      link_l2: 0.18
```

- [ ] **Step 8: Write the BT client node**

Create `plugins/action/decision_action_client_node.cpp` and its header following the `ArmActionClientNode` shape from Task 4 Step 7, registering XML tag `MakeDecisionClient` against action name `decision_action`. In `onResultReceived`, `setOutput` each of the ports already declared in `make_decision_client_node.hpp:19-42` from the corresponding result field.

- [ ] **Step 9: Retire the old node**

```bash
git rm ros_env/src/bizon_behavior_clients/plugins/action/make_decision_client_node.cpp \
       ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/action/make_decision_client_node.hpp
```
Remove its `add_library`/`list(APPEND plugin_libs ...)` block from `bizon_behavior_clients/CMakeLists.txt` and its entry from `plugin_lib_names_` in `src/main.cpp`, replacing both with `decision_action_client_node`.

- [ ] **Step 10: Build and verify in simulation**

```bash
cd ros_env
colcon build --packages-up-to bizon_behavior_clients bizon_behavior_servers && . install/setup.bash
ros2 action list | grep decision_action
```
Expected: `/bizon2/decision_action`. Then run the full stack and confirm a white move still executes end to end, and that `ros2 node list` shows **no** Stockfish-owning process other than `behavior_server`.

- [ ] **Step 11: Commit**

```bash
git add -A ros_env/src
git commit -m "refactor(decision): move Stockfish behind a timed decision_action server"
```

---

### Task 6: Real lifecycle management and the E-stop path

Closes F5.

**Files:**
- Modify: `ros_env/src/bizon_lifecycle_manager/src/lifecycle_manager.cpp:33-39`
- Create: `ros_env/src/bizon_behavior_clients/include/bizon_behavior_clients/plugins/condition/is_system_active_node.hpp`
- Create: `ros_env/src/bizon_behavior_clients/plugins/condition/is_system_active_node.cpp`
- Modify: `ros_env/src/bizon_behavior_clients/CMakeLists.txt`, `src/main.cpp`
- Modify: `ros_env/src/bizon_behavior_clients/behavior_trees/chess_game.xml`
- Modify: `ros_env/src/bizon_player_bringup/launch/bizon_lifecycle_dev.launch.py` (pass `node_names`)

**Interfaces:**
- Consumes: `bizon_lifecycle_manager::LifecycleManagerClient` (already exists at `lifecycle_manager_client.hpp`, currently unused) and its `is_active()` returning `SystemStatus`.
- Produces: BT condition node `IsSystemActive` with input port `lifecycle_manager_name` (string, default `lifecycle_manager`).

- [ ] **Step 1: Un-hardcode the managed node list**

Replace `lifecycle_manager.cpp:33-39`:
```cpp
    node_names_ = get_parameter("node_names").as_string_array();
    RCLCPP_INFO(
        get_logger(),
        "Managing %zu lifecycle nodes",
        node_names_.size());
    for (const auto & name : node_names_) {
        RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
    }
```

- [ ] **Step 2: Pass the list from the launch file**

In `bizon_player_bringup/launch/bizon_lifecycle_dev.launch.py`, ensure the lifecycle_manager node receives:
```python
{'node_names': ['behavior_server'], 'autostart': True}
```
`behavior_server` now hosts wait, board, arm, and decision behaviors, so managing it manages all four. When the arm and decision plugins are later split into their own lifecycle nodes for the Jetson, add their names to this list — that is the only change required.

- [ ] **Step 3: Write the condition node**

`ros_env/src/bizon_behavior_clients/plugins/condition/is_system_active_node.cpp`:
```cpp
#include "bizon_behavior_clients/plugins/condition/is_system_active_node.hpp"

namespace bizon_behavior_clients
{

IsSystemActiveNode::IsSystemActiveNode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string manager_name = "lifecycle_manager";
  getInput("lifecycle_manager_name", manager_name);

  client_ = std::make_shared<bizon_lifecycle_manager::LifecycleManagerClient>(
    manager_name, node_);
}

BT::NodeStatus IsSystemActiveNode::tick()
{
  const auto status = client_->is_active(std::chrono::seconds(1));
  if (status == bizon_lifecycle_manager::SystemStatus::ACTIVE) {
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "Lifecycle manager reports the system is not active; refusing to command the arm");
  return BT::NodeStatus::FAILURE;
}

}  // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bizon_behavior_clients::IsSystemActiveNode>("IsSystemActive");
}
```

Header `is_system_active_node.hpp` declares the class with members `rclcpp::Node::SharedPtr node_`, `std::shared_ptr<bizon_lifecycle_manager::LifecycleManagerClient> client_`, and:
```cpp
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("lifecycle_manager_name", "lifecycle_manager",
      "Name of the lifecycle manager to query")};
  }
```

Add `bizon_lifecycle_manager` to `find_package`, `dependencies`, and `package.xml` of `bizon_behavior_clients`; add the `is_system_active_node` library to `CMakeLists.txt` and to `plugin_lib_names_` in `main.cpp`.

- [ ] **Step 4: Gate the work branch on system health**

In `chess_game.xml`, wrap the `PlayOneMove` sequence's first element by inserting immediately after `<Sequence name="PlayOneMove">`:
```xml
                <IsSystemActive lifecycle_manager_name="lifecycle_manager" />
```

- [ ] **Step 5: Build and verify the E-stop path**

```bash
cd ros_env
colcon build --packages-up-to bizon_behavior_clients && . install/setup.bash
```

Run the full stack, then mid-game:
```bash
ros2 service call /lifecycle_manager/manage_nodes bizon_msgs/srv/ManageLifecycleNodes "{command: 1}"   # PAUSE
```
Expected: `behavior_server` deactivates, the next tree tick logs `"refusing to command the arm"`, and the tree stops issuing arm goals instead of continuing blind.

Then:
```bash
ros2 service call /lifecycle_manager/manage_nodes bizon_msgs/srv/ManageLifecycleNodes "{command: 2}"   # RESUME
```
Expected: the system reactivates and play resumes.

- [ ] **Step 6: Commit**

```bash
git add ros_env/src/bizon_lifecycle_manager ros_env/src/bizon_behavior_clients ros_env/src/bizon_player_bringup
git commit -m "feat(lifecycle): parameterize managed nodes and gate the tree on system health"
```

---

### Task 7: Planning scene collision objects from the board state

> **REJECTED BY DESIGN, 2026-08-23. Do not implement.** F6 called the empty planning
> scene a defect. It is not: this robot is vision-driven and deliberately carries no
> collision model of the board. A camera reports where the pieces are, the arm goes to
> that x/y and picks. Pieces are avoided by the motion pattern — rise to the clearance
> height, traverse, descend — which is what `MoveSequence`'s "up" waypoints exist for,
> not by collision checking. The real robot has a gripper and an RGB webcam and nothing
> else; there is no sensor that could keep a collision model honest.
>
> This was implemented once and reverted. It put 32 cylinders into the planning scene,
> MoveIt rejected the goal states, and the game ended with `RecoveryNode` exhausting its
> retries on the first move. The commits are kept on branch
> `task-7-planning-scene-rejected` as a record, not as work to resume.

Closes F6.

**Files:**
- Modify: `ros_env/src/bizon_behavior_servers/include/bizon_behavior_servers/plugins/arm_plugin.hpp`
- Modify: `ros_env/src/bizon_behavior_servers/plugins/arm_plugin.cpp`
- Create: `ros_env/src/bizon_chess/include/bizon_chess/fen.hpp`, `src/fen.cpp`
- Test: `ros_env/src/bizon_chess/test/test_fen.cpp`

**Interfaces:**
- Consumes: `bizon_chess::squareToWorld` from Task 1; the `fen` blackboard entry produced by `BoardActionIsaacClient`.
- Produces: `bizon_chess::occupiedSquares(const std::string& fen) -> std::vector<std::string>`, and an `Arm.action` goal field `string board_fen` used to refresh the scene before planning.

- [ ] **Step 1: Write the failing FEN test**

`ros_env/src/bizon_chess/test/test_fen.cpp`:
```cpp
#include <gtest/gtest.h>
#include <algorithm>
#include <string>
#include <vector>

#include "bizon_chess/fen.hpp"

using bizon_chess::occupiedSquares;

namespace {
bool contains(const std::vector<std::string> & v, const std::string & s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}
}  // namespace

TEST(Fen, OpeningPositionHasThirtyTwoOccupiedSquares)
{
  const auto squares =
    occupiedSquares("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_EQ(squares.size(), 32u);
  EXPECT_TRUE(contains(squares, "a1"));
  EXPECT_TRUE(contains(squares, "e8"));
  EXPECT_TRUE(contains(squares, "h2"));
  EXPECT_FALSE(contains(squares, "e4"));
}

TEST(Fen, EmptyBoardHasNoOccupiedSquares)
{
  EXPECT_TRUE(occupiedSquares("8/8/8/8/8/8/8/8 w - - 0 1").empty());
}

TEST(Fen, SinglePieceIsPlacedOnTheCorrectSquare)
{
  // Rank 8 is the first field; four empty files then a white king on e8.
  const auto squares = occupiedSquares("4K3/8/8/8/8/8/8/8 w - - 0 1");
  ASSERT_EQ(squares.size(), 1u);
  EXPECT_EQ(squares[0], "e8");
}

TEST(Fen, HandlesBoardOnlyFenWithoutTrailingFields)
{
  const auto squares = occupiedSquares("8/8/8/4p3/8/8/8/8");
  ASSERT_EQ(squares.size(), 1u);
  EXPECT_EQ(squares[0], "e5");
}

TEST(Fen, ReturnsEmptyForMalformedInput)
{
  EXPECT_TRUE(occupiedSquares("").empty());
  EXPECT_TRUE(occupiedSquares("not-a-fen").empty());
}
```

Add to `bizon_chess/CMakeLists.txt`: `src/fen.cpp` in the library sources, and
```cmake
  ament_add_gtest(test_fen test/test_fen.cpp)
  target_link_libraries(test_fen ${PROJECT_NAME})
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd ros_env && colcon build --packages-select bizon_chess
```
Expected: FAIL — `bizon_chess/fen.hpp: No such file or directory`.

- [ ] **Step 3: Implement `occupiedSquares`**

`ros_env/src/bizon_chess/include/bizon_chess/fen.hpp`:
```cpp
#ifndef BIZON_CHESS__FEN_HPP_
#define BIZON_CHESS__FEN_HPP_

#include <string>
#include <vector>

namespace bizon_chess
{
/// Algebraic names of every occupied square in a FEN's board field.
/// Accepts a full FEN or a bare board field. Returns empty on malformed input.
std::vector<std::string> occupiedSquares(const std::string & fen);
}  // namespace bizon_chess
#endif  // BIZON_CHESS__FEN_HPP_
```

`ros_env/src/bizon_chess/src/fen.cpp`:
```cpp
#include "bizon_chess/fen.hpp"

#include <cctype>

namespace bizon_chess
{

std::vector<std::string> occupiedSquares(const std::string & fen)
{
  std::vector<std::string> squares;
  if (fen.empty()) {
    return squares;
  }

  const std::string board = fen.substr(0, fen.find(' '));

  int rank = 8;
  int file = 0;
  for (const char c : board) {
    if (c == '/') {
      if (file != 8) { return {}; }   // short or long rank: malformed
      rank--;
      file = 0;
      if (rank < 1) { return {}; }
    } else if (c >= '1' && c <= '8') {
      file += c - '0';
      if (file > 8) { return {}; }
    } else if (std::isalpha(static_cast<unsigned char>(c))) {
      if (file > 7) { return {}; }
      squares.push_back(std::string(1, static_cast<char>('a' + file)) + std::to_string(rank));
      file++;
    } else {
      return {};
    }
  }

  if (rank != 1 || file != 8) {
    return {};
  }
  return squares;
}

}  // namespace bizon_chess
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd ros_env && colcon build --packages-select bizon_chess && colcon test --packages-select bizon_chess && colcon test-result --verbose
```
Expected: 10 tests total (5 geometry + 5 FEN), all PASS.

- [ ] **Step 5: Add `board_fen` to the Arm action and refresh the scene**

Add to the goal section of `ros_env/src/bizon_msgs/action/Arm.action`:
```
string board_fen
```

In `ArmPlugin::onConfigure`, add a `moveit::planning_interface::PlanningSceneInterface planning_scene_;` member and a `bizon_chess::RobotParams params_` filled from the same parameters as `DecisionPlugin`.

In `ArmPlugin::onRun`, before setting the joint target, rebuild the scene:
```cpp
  // Publish the board and every occupied square as collision objects so the
  // planner routes around pieces instead of sweeping through them.
  if (!command->board_fen.empty()) {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (const auto & square : bizon_chess::occupiedSquares(command->board_fen)) {
      double x = 0.0;
      double y = 0.0;
      if (!bizon_chess::squareToWorld(square, params_, x, y)) {
        continue;
      }

      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id = move_group_arm_->getPlanningFrame();
      obj.id = "piece_" + square;
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;

      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions = {piece_height_, piece_radius_};
      obj.primitives.push_back(primitive);

      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = x - params_.robot_base_offset_x;
      pose.position.y = y;
      pose.position.z = piece_height_ / 2.0;
      obj.primitive_poses.push_back(pose);

      objects.push_back(obj);
    }
    planning_scene_.applyCollisionObjects(objects);
  }
```

Declare `piece_height_` (default 0.03) and `piece_radius_` (default 0.012) as ROS parameters under `arm_action.`.

The piece being moved must be removed from the scene before the pick, or the planner will refuse to approach it. In `onRun`, when `command->board_fen` is non-empty, also remove the object whose id matches the square the arm is descending onto — pass that square as a new goal field `string target_square` and call `planning_scene_.removeCollisionObjects({"piece_" + command->target_square})`.

- [ ] **Step 6: Pass the FEN and target square from the tree**

Add ports `board_fen` and `target_square` to `ArmActionClientNode::providedPorts()` and copy them into the goal in `on_tick()`. In `chess_game.xml`, add `board_fen="{fen}"` to every `ArmActionClient` in `MoveSequence`, and `target_square` on the two descending moves.

- [ ] **Step 7: Verify in simulation**

Run the full stack. In RViz, add a `PlanningScene` display.
Expected: 32 cylinders appear on the board at game start and shrink to 31 after the first capture. Command a move whose straight-line path crosses an occupied square and confirm the planned trajectory arcs over it rather than through it.

- [ ] **Step 8: Commit**

```bash
git add -A ros_env/src
git commit -m "feat(arm): populate the planning scene from the board FEN"
```

---

## Deferred to the Hardware Phase

Recorded here so they are not lost, but explicitly **not** in this plan:

- `"serial"` branch in `bizon_system_interface.cpp:66-95`, with an async I/O thread so `read()` never blocks the 100 Hz `controller_manager` loop.
- Gripper controller switched from `FollowJointTrajectory` to `control_msgs/action/GripperCommand` with `max_effort`, and `effort` added to `hand_group_controller.state_interfaces` — the real fix for F1's root cause. Task 3 makes the *consequence* safe; this makes the stall stop being an error at all.
- Hand-eye calibration publishing the board pose to TF, replacing `RobotParams` defaults.
- `on_deactivate()` in `bizon_system_interface.cpp:113` commanding a real safe state.
- YOLO fine-tuning on real images plus multi-frame FEN voting.
- Jetson tuning: `ARCH=armv8` Stockfish build, engine thread/hash limits (already parameterized in Task 5), and replacing OMPL with direct joint interpolation for the vertical approach and retreat moves.

## Notes for the Executor

- `bizon_behavior_servers/CMakeLists.txt:104` registers `"bizon_behavior_servers::BehaviorServer"` but `src/behavior_server.cpp:8` declares `namespace behavior_server`. The component is never loaded by name today so this is latent, but fix it in whichever task touches that file first.
- `bizon_behavior_clients/CMakeLists.txt` calls `add_executable(bizon_behavior_tree_client_main ...)` and links `${library_name}` **before** `add_library(${library_name} ...)`. CMake tolerates this, but move the `add_library` above the `add_executable` when editing that file.
- Every task that adds a plugin must update **both** `bizon_behavior_params.yaml` and `bizon_behavior_params_bizon3.yaml`, or the black robot silently loses the behavior.
