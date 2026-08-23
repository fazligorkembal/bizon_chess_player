#ifndef BIZON_BEHAVIOR_SERVERS__PLUGINS__DECISION_PLUGIN_HPP_
#define BIZON_BEHAVIOR_SERVERS__PLUGINS__DECISION_PLUGIN_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "bizon_behavior_servers/debug_session.hpp"
#include "bizon_behavior_servers/stockfish_process.hpp"
#include "bizon_behavior_servers/timed_behavior.hpp"
#include "bizon_chess/board_geometry.hpp"
#include "bizon_chess/robot_params.hpp"
#include "bizon_msgs/action/decision.hpp"

namespace bizon_behaviors
{
/// Stockfish-backed move decision -- the successor to the old MakeDecisionNode
/// SyncActionNode (bizon_behavior_clients/plugins/action/make_decision_client_node.*,
/// deleted in the same change that added this file). The engine now runs
/// behind a StockfishProcess with a deadline on every read (closes F4: a
/// wedged engine used to block the whole behavior tree, with the arm
/// potentially mid-air) and the decision runs behind an action server rather
/// than inline in a BT tick (completes F7).
///
/// onRun validates the goal and starts the engine if it is not already
/// running. onCycleUpdate performs the whole decision -- move-owner
/// detection, and if it is our move, the search and the resulting arm
/// geometry -- in one call and returns a terminal status; there is no
/// multi-cycle RUNNING phase. Every Stockfish round trip inside
/// StockfishProcess already carries its own bounded deadline (search_timeout_),
/// so a single onCycleUpdate call is itself bounded. This differs from
/// ArmPlugin, whose motion can legitimately run for seconds and so is
/// launched with std::async and polled across many cycles instead.
class DecisionPlugin : public TimedBehavior<bizon_msgs::action::Decision>
{
public:
  using DecisionAction = bizon_msgs::action::Decision;

  DecisionPlugin();
  ~DecisionPlugin() override;

  void onConfigure() override;
  void onCleanup() override;

  ResultStatus onRun(const std::shared_ptr<const DecisionAction::Goal> command) override;
  ResultStatus onCycleUpdate() override;
  void onActionCompletion(std::shared_ptr<DecisionAction::Result> result) override;

private:
  // ---- Ported 1:1 from MakeDecisionNode (pre-Task-5
  // make_decision_client_node.cpp); see that file's git history for the
  // BT-node version of each. ----
  std::string get_last_move_from_text();
  bool write_to_text_file(const std::string & fen);
  std::vector<std::string> possible_next_moves_from_valid_fen(const std::string & fen);
  bool apply_move_to_fen(const std::string & base_fen, const std::string & move, std::string & fen_out);
  std::string who_is_owner_of_move();
  // Commits fen_pending_ to the history file if the camera agrees with it,
  // either directly or one ply on. Returns true when the history was
  // advanced, in which case fen_from_text_ has been refreshed.
  bool confirm_pending_against_camera();
  bool is_checkmate(const std::string & fen);
  bool get_best_move(const std::string & fen, std::string & move_out);
  void reset_values();

  // DecisionPlugin has no images of its own -- see debug_session.hpp's
  // class comment on the cross-plugin seam. This opens an error bundle via
  // DebugSession, writes context_text (already including the square-level
  // diff, when the caller has both FENs) as context.txt, and asks
  // BoardPlugin's registered callback to fill in the images. Best-effort:
  // never throws, and a failure here never fails the goal.
  void captureFailureBundle(const std::string & label, const std::string & context_text);

  // ---- onCycleUpdate's three top-level branches, split out of one
  // function only for readability; each returns the terminal ResultStatus
  // for that branch exactly as the corresponding block of the old tick()
  // would have. ----
  ResultStatus handle_save_piece();
  ResultStatus handle_checkmate(const std::string & fen);
  ResultStatus handle_move();

  // handle_move()'s per-move-type geometry. Each fills the move_from*_/
  // move_to*_ members for the pieces it uses and returns false if any
  // target is outside the arm's reachable annulus (see
  // joint_targets_from_xy()).
  bool handle_straight();
  bool handle_two_piece_move();  // castle, capture, en_passant, promotion
  bool handle_promotion_capture();

  // ---- Geometry helpers. squareToWorld()/worldToJointAngles() come from
  // bizon_chess and know nothing about which side the robot plays;
  // mirroring for the black-side robot is applied here, matching the
  // "if (player_side_ == white) ... else ..." block that used to be
  // repeated at every call site in tick(). Several move types add a
  // constant offset to a box's Y coordinate (parking captured pieces,
  // clearing the promotion pile, ...); WHERE that happens relative to
  // mirroring differs by move type in the pre-refactor code and is
  // preserved exactly at each call site -- see handle_two_piece_move()'s
  // and handle_promotion_capture()'s comments. ----
  bool square_to_world(const std::string & box, double & x, double & y);
  void mirror_xy(double & x, double & y) const;
  bool box_to_world_mirrored(const std::string & box, double & x, double & y);
  // A false return means the target is outside the arm's reachable
  // annulus. The pre-refactor calculate_joint_angles() fed this straight
  // into sqrtf(1 - d*d) and produced NaN joint targets that the arm would
  // then be commanded to; worldToJointAngles() catches it instead, and
  // every caller here turns it into error_code 3.
  bool joint_targets_from_xy(
    double x, double y, std::vector<double> & up_out, std::vector<double> & down_out);

  bool ensure_engine_running();

  // Search depth plus the two UCI options that tune how hard the engine
  // tries: skill_level (0-20, engine plays deliberately worse below 20) and
  // multipv (how many candidate lines it ranks, which slows and broadens
  // its choice among near-equal moves). One of these is looked up per
  // search by player_side_ -- see get_best_move().
  struct EngineStrength
  {
    int search_depth;
    int skill_level;
    int multipv;
  };

  // ---- Configuration: bizon_chess::RobotParams and the decision-specific
  // knobs, all read from decision_action.* ROS parameters in onConfigure().
  // Replaces the constants at the old make_decision_client_node.hpp:103-110
  // and the depth/timeout/skill-level magic numbers inline in the old
  // get_best_move(). ----
  bizon_chess::RobotParams robot_params_;
  // The old get_best_move() deliberately handicapped one side (white:
  // Skill Level 5, MultiPV 3, depth 10) while the other searched at full
  // engine strength (black: depth 20, and -- because it never sent
  // "setoption name Skill Level"/"MultiPV" at all -- the engine's own
  // defaults, Skill Level 20 and MultiPV 1). That is not dead configuration
  // to unify away: this stack is heading for a single arm against a human
  // opponent, so how hard the robot plays is a knob its operator tunes per
  // robot/side, not something a refactor should decide. Both sides are
  // parameterized identically (decision_action.white.*/decision_action.black.*)
  // so either can be retuned without a code change; the defaults below
  // reproduce the old hardcoded values bit for bit.
  EngineStrength white_strength_{10, 5, 3};
  EngineStrength black_strength_{20, 20, 1};
  std::chrono::milliseconds search_timeout_{10000};
  int engine_threads_{1};
  int engine_hash_mb_{16};
  // May contain the literal token "<player_side>", substituted in onRun()
  // once the goal's player_side is known -- see state_file_.
  std::string state_file_template_;

  StockfishProcess engine_;

  // ---- Per-goal state, set in onRun() ----
  std::string player_side_;
  std::string opponent_side_;
  // state_file_template_ with "<player_side>" substituted. Replaces the old
  // hardcoded "/home/user/Documents/bizon_chess_player/<side>_last_moves.txt".
  std::string state_file_;
  std::string fen_from_camera_;

  // ---- Scratch, cleared at the top of every onCycleUpdate() by
  // reset_values() ----
  std::string fen_from_text_;
  std::string fen_from_text_only_board_;
  std::string fen_validated_;
  std::string move_owner_detected_;
  std::string move_best_;
  int count_white_captured_{0};
  int count_black_captured_{0};

  // ---- Persistent across goals, exactly like the fields on the old
  // long-lived SyncActionNode instance: DecisionPlugin is likewise
  // constructed once by the behavior server and reused for every goal. ----
  //
  // The position we expect the board to be in once the move we just handed
  // back has actually been executed. Nothing else writes our own move into
  // the history file: onCycleUpdate decides and returns without ever
  // learning whether the arm succeeded, so writing this optimistically here
  // would put the file *ahead* of the board, and reconciliation
  // (who_is_owner_of_move()'s search ladder) only ever searches forward --
  // an ahead-of-reality file can never be reconciled and the game
  // deadlocks. So it is held here in memory and committed only against
  // camera evidence on the next goal. See confirm_pending_against_camera().
  std::string fen_pending_;
  int count_saved_promotion_pieces_{2};
  int count_promotion_{0};

  // ---- Result, filled during onCycleUpdate() and copied into the actual
  // DecisionAction::Result by onActionCompletion() -- TimedBehavior::execute()
  // constructs the Result object itself and only hands it to
  // onActionCompletion() at the very end, so there is nowhere earlier to
  // write into it directly. Matches BoardPlugin's result_-in-a-member
  // pattern. ----
  std::string move_type_;
  int32_t move_count_{0};
  std::vector<double> hand_open_position_;
  std::vector<double> hand_close_position_;
  std::vector<double> move_from1_, move_from_down1_, move_to1_, move_to_down1_;
  std::vector<double> move_from2_, move_from_down2_, move_to2_, move_to_down2_;
  std::vector<double> move_from3_, move_from_down3_, move_to3_, move_to_down3_;
};
}  // namespace bizon_behaviors

#endif  // BIZON_BEHAVIOR_SERVERS__PLUGINS__DECISION_PLUGIN_HPP_
