# Simulation Verification Checklist — Pre-Hardware Refactor

These are the steps from `2026-08-19-pre-hardware-refactor.md` that cannot be run from an
automated session: each needs a live Isaac Sim GUI plus a person watching for a physical
outcome. Everything else in each task (code, unit tests, compile verification) is verified
before the task is marked complete.

Run these when Isaac Sim is up. Each entry names the task it belongs to, what to run, and the
exact thing to look for.

## Common startup

Terminal A (host):
```bash
./isaac_sim_standalone.sh
```

Terminals B–D (inside the container — `docker compose -f docker/docker-compose-x86.yml run bizon_chess_player bash`):
```bash
ros2 launch bizon_player_bringup bizon_player_bringup.launch.py prefix:=bizon2
ros2 launch bizon_player_bringup bizon_lifecycle_dev.launch.py namespace:=bizon2
ros2 run bizon_behavior_clients bizon_behavior_tree_client_main --ros-args -r __ns:=/bizon2 -p use_sim_time:=true
```

---

## [ ] Task 2 — arm and gripper are serialized

**Plan reference:** Task 2, Step 6.

Run one full white move.

**Pass:** for every `MoveSequence` step the log shows `"Arm movement started"`, and the gripper
log line appears only *after* the arm future resolves. The two never interleave. The piece ends
on the target square.

**Fail signal:** any log where a hand move begins while an arm move is still outstanding. That is
exactly the concurrency that knocks pieces over on real hardware.

---

## [ ] Task 3 — recovery releases before it retreats

**Plan reference:** Task 3, Step 9.

Force a failure mid-`MoveSequence`: pause Isaac Sim's physics while the arm is descending, so
MoveIt reports a failure.

**Pass:** the log shows, in this order:
```
[RecoveryNode] work branch failed, running recovery (attempt 1/3)
... ArmActionClient ... hand_open_position ...
... ArmActionClient ... recovery_lift_position ...
... ArmActionClient ... home_position ...
```
On screen the gripper opens **before** the arm translates, and no piece is dragged across the
board.

**Fail signal:** the arm moves toward home while still holding a piece. This is the specific
disaster the whole task exists to prevent — treat it as blocking.

---

## [ ] Task 4 — arm motion runs through an action server

**Plan reference:** Task 4, Step 9.

```bash
ros2 action list | grep arm_action
```
**Pass:** `/bizon2/arm_action` is listed. One full white move executes exactly as before.

Then, mid-game:
```bash
ros2 lifecycle set /bizon2/behavior_server deactivate
```
**Pass:** the arm stops accepting goals. Before this task it would have kept moving.

---

## [ ] Task 6 — the lifecycle manager is a working E-stop

**Plan reference:** Task 6, Step 5.

Mid-game:
```bash
ros2 service call /lifecycle_manager/manage_nodes bizon_msgs/srv/ManageLifecycleNodes "{command: 1}"   # PAUSE
```
**Pass:** `behavior_server` deactivates, the next tree tick logs `"refusing to command the arm"`,
and the tree stops issuing arm goals rather than continuing blind.

```bash
ros2 service call /lifecycle_manager/manage_nodes bizon_msgs/srv/ManageLifecycleNodes "{command: 2}"   # RESUME
```
**Pass:** the system reactivates and play resumes.

---

## [ ] Task 7 — the planner sees the pieces

**Plan reference:** Task 7, Step 7.

In RViz, add a `PlanningScene` display.

**Pass:** 32 cylinders appear on the board at game start and drop to 31 after the first capture.
Command a move whose straight-line path crosses an occupied square and confirm the planned
trajectory arcs over it rather than through it.

**Fail signal:** no collision objects, or a trajectory that passes through a cylinder — the
planner is not seeing the board and will knock pieces on real hardware.

---

## Reporting back

For each item, paste the relevant log lines (or say what you saw) into the session. A failure
here reopens its task rather than blocking the ones after it, since these run as a batch after
the code work is complete.
