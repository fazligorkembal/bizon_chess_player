# bizon_chess_player

A ROS 2 Humble robot that plays physical chess. A camera reads the board, Stockfish
picks the move, a behavior tree sequences the game, and an arm executes it.

## Layout

- `ros_env/` is the colcon workspace — **not** the repo root. Packages live in `ros_env/src/`.
- `ros_env/src/BehaviorTree.CPP/` is vendored upstream code, gitignored, and about 104k
  of the repo's ~118k lines. First-party code is the `bizon_*` packages, roughly 14k lines.
- `Stockfish/`, `models/`, `assets/`, and the weight files (`*.pt`, `*.wts`, `*.onnx`,
  `*.engine`) are gitignored.
- `white_last_moves.txt` / `black_last_moves.txt` are runtime state written during play.
  Never commit them.

## Building and testing

There is no ROS on the host. Everything builds inside the image defined by
`docker/dockerfile-amd64`, brought up through `docker/docker-compose-x86.yml`
(service `bizon_chess_player`; the repo mounts at
`/home/user/Documents/bizon_chess_player`).

Use the wrapper, not `colcon` directly:

```
scripts/rosbuild [colcon build args]   # build; prints one line on success
scripts/rosbuild --test [args]         # colcon test + colcon test-result
scripts/rosbuild --log                 # path of the last full log
scripts/rosbuild --explain             # local LLM pins the root cause of the last failure
```

Arguments pass through, so `scripts/rosbuild --packages-select bizon_behavior_clients`
works. On failure it prints the failing packages, the decisive error lines, and a short
tail; the complete log is always kept at `~/.cache/bizon-build/last.log`. A raw colcon
log is thousands of lines and reading one wholesale is the most wasteful thing an agent
can do in this repo — read the full log only when the summary genuinely is not enough.

## Invariants worth knowing before changing behavior

The full reasoning lives in the project's ADR (`manage_adr` in codebase-memory). Two
rules are load-bearing and easy to break by accident:

1. **The move-history file must never describe a position the camera has not confirmed.**
   `MakeDecisionNode` is a `SyncActionNode` and never learns whether the arm succeeded,
   so its own move is held in `fen_pending_` and committed only against camera evidence.
   Writing it optimistically puts the file ahead of the board, and since reconciliation
   only searches forward, the game deadlocks permanently.
2. **Only genuine faults belong in a recovery branch.** Waiting for the opponent is a
   normal steady state: it returns `SUCCESS` with `move_type` "wait", and the tree's
   `MoveOrWaitForOpponent` fallback skips the move subtree. Returning `FAILURE` there
   exhausts `RecoveryNode`'s retries and abandons the game mid-play.
3. **The robot carries no collision model, and that is deliberate.** It is vision-driven:
   the camera says where the pieces are, the arm goes to that x/y and picks. Pieces are
   avoided by the motion pattern — rise to the clearance height, traverse, descend — not
   by collision checking. The real robot has a gripper and an RGB webcam and nothing else.
   Populating the planning scene from the board state has been tried and reverted; MoveIt
   rejected every goal and the game ended on the first move.

Also: the arm and gripper are moved in sequence, never together, because simultaneous
motion disturbs a held piece. `hand_only` goals skip the arm entirely.

## Finding things

The repository is indexed in the codebase-memory MCP under
`home-gorkem-Documents-projects-bizon_chess_player`. Answer structural questions —
callers, definitions, blast radius, module seams — from the graph rather than by reading
files. It covers first-party code only; BehaviorTree.CPP internals still need file reads,
and a handful of files have unparsed line ranges where grep is the honest tool.

## Looking things up

For CUDA and TensorRT, use the nvidia-cuda-docs MCP. Engine building here is version- and
geometry-sensitive; `CellClassifier`'s constructor arguments must mirror the weights file,
not be tuned by hand.

## Conventions

- Conventional commits with a scope: `feat(servers):`, `fix(decision):`, `build(docker):`.
- Comments explain *why*, especially where a simpler-looking alternative is wrong. Match
  that density; the existing comments are load-bearing documentation.
- Nodes needing a live ROS graph get structural tests that parse the shipped XML
  (`test_behavior_plugin_xml.cpp`, `test_chess_game_tree_structure.cpp`). Logic that can
  be tested directly should be extracted so that it is — see `fen_utils.hpp`.
- Pin external sources deliberately. An unpinned OpenCV clone silently moved to 5.x and
  broke every `cv_bridge` dependant.
