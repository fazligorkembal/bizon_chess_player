#include "bizon_behavior_servers/plugins/decision_plugin.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "bizon_chess/fen_utils.hpp"
#include "bizon_chess/move_classification.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bizon_behaviors
{

namespace
{

// Ported from make_decision_client_node.cpp. Detects OCR misclassification:
// the camera can confuse a piece's colour without confusing its square, so a
// FEN comparison that only cares about occupied-vs-empty (not which side
// owns each square) can still match when the strict comparison does not.
//
// DEBUG, not ERROR: this runs once per candidate inside the recovery
// ladder's search loops in who_is_owner_of_move(), so at ERROR it floods the
// console with what is ordinary search progress and buries real failures.
bool compare_fens_without_classes(const std::string & fen_camera, const std::string & fen_text)
{
  auto replace_letters = [](const std::string & fen) {
    std::string result;
    for (char c : fen) {
      if (std::isupper(static_cast<unsigned char>(c))) {
        result += 'X';
      } else if (std::islower(static_cast<unsigned char>(c))) {
        result += 'x';
      } else {
        result += c;
      }
    }
    return result;
  };
  const std::string masked_camera = replace_letters(fen_camera);
  const std::string masked_text = replace_letters(fen_text);
  RCLCPP_DEBUG(
    rclcpp::get_logger("DecisionPlugin"),
    "Comparing FENs without classes. Camera: %s Text: %s -> masked %s vs %s",
    fen_camera.c_str(), fen_text.c_str(), masked_camera.c_str(), masked_text.c_str());
  return masked_camera == masked_text;
}

// Ported from make_decision_client_node.cpp's get_king_square(). Finds the
// king belonging to whoever moves at `fen`, via bizon_chess::side_to_move_field
// rather than the ad hoc substring pull the pre-refactor free function used.
std::string king_square(const std::string & fen)
{
  const std::string side = bizon_chess::side_to_move_field(fen);
  const char king = (side == "w") ? 'K' : 'k';
  const std::string board = fen.substr(0, fen.find(' '));

  int rank = 8;
  int file = 0;
  for (char c : board) {
    if (c == '/') {
      rank--;
      file = 0;
    } else if (c >= '1' && c <= '8') {
      file += c - '0';
    } else {
      if (c == king) {
        return std::string(1, static_cast<char>('a' + file)) + std::to_string(rank);
      }
      file++;
    }
  }
  return "";
}

// Ported unchanged from make_decision_client_node.cpp. Used to pick which of
// the four parking slots per side (h1..h4 / a9-1..a9-4, i.e. a6..a9 folded)
// a captured piece goes into, indexed by how many pieces of that colour are
// already off the board.
void get_captured_pieces_count(
  const std::string & board_only_fen, int & white_captured_count, int & black_captured_count)
{
  white_captured_count = 0;
  black_captured_count = 0;
  for (char c : board_only_fen) {
    if (c >= 'A' && c <= 'Z') {
      white_captured_count++;
    } else if (c >= 'a' && c <= 'z') {
      black_captured_count++;
    }
  }
  white_captured_count = 16 - white_captured_count;
  black_captured_count = 16 - black_captured_count;
}

}  // namespace

DecisionPlugin::DecisionPlugin() = default;
DecisionPlugin::~DecisionPlugin() = default;

void DecisionPlugin::onConfigure()
{
  auto node = node_.lock();

  // Per-side engine strength -- see the EngineStrength/white_strength_/
  // black_strength_ comments in the header for why this is not a single
  // flat search_depth. Selected at search time by player_side_, in
  // get_best_move().
  if (!node->has_parameter(behavior_name_ + ".white.search_depth")) {
    node->declare_parameter(behavior_name_ + ".white.search_depth", white_strength_.search_depth);
  }
  node->get_parameter(behavior_name_ + ".white.search_depth", white_strength_.search_depth);
  if (!node->has_parameter(behavior_name_ + ".white.skill_level")) {
    node->declare_parameter(behavior_name_ + ".white.skill_level", white_strength_.skill_level);
  }
  node->get_parameter(behavior_name_ + ".white.skill_level", white_strength_.skill_level);
  if (!node->has_parameter(behavior_name_ + ".white.multipv")) {
    node->declare_parameter(behavior_name_ + ".white.multipv", white_strength_.multipv);
  }
  node->get_parameter(behavior_name_ + ".white.multipv", white_strength_.multipv);

  if (!node->has_parameter(behavior_name_ + ".black.search_depth")) {
    node->declare_parameter(behavior_name_ + ".black.search_depth", black_strength_.search_depth);
  }
  node->get_parameter(behavior_name_ + ".black.search_depth", black_strength_.search_depth);
  if (!node->has_parameter(behavior_name_ + ".black.skill_level")) {
    node->declare_parameter(behavior_name_ + ".black.skill_level", black_strength_.skill_level);
  }
  node->get_parameter(behavior_name_ + ".black.skill_level", black_strength_.skill_level);
  if (!node->has_parameter(behavior_name_ + ".black.multipv")) {
    node->declare_parameter(behavior_name_ + ".black.multipv", black_strength_.multipv);
  }
  node->get_parameter(behavior_name_ + ".black.multipv", black_strength_.multipv);

  double search_timeout_s = 10.0;
  if (!node->has_parameter(behavior_name_ + ".search_timeout")) {
    node->declare_parameter(behavior_name_ + ".search_timeout", search_timeout_s);
  }
  node->get_parameter(behavior_name_ + ".search_timeout", search_timeout_s);
  search_timeout_ = std::chrono::milliseconds(static_cast<int64_t>(search_timeout_s * 1000.0));

  if (!node->has_parameter(behavior_name_ + ".engine_threads")) {
    node->declare_parameter(behavior_name_ + ".engine_threads", engine_threads_);
  }
  node->get_parameter(behavior_name_ + ".engine_threads", engine_threads_);

  if (!node->has_parameter(behavior_name_ + ".engine_hash_mb")) {
    node->declare_parameter(behavior_name_ + ".engine_hash_mb", engine_hash_mb_);
  }
  node->get_parameter(behavior_name_ + ".engine_hash_mb", engine_hash_mb_);

  // RobotParams fields. Only a subset of these (box_size,
  // robot_base_offset_x, link_l1, link_l2) are in the shipped param files;
  // the rest keep RobotParams's own defaults, which are the values that
  // used to be hardcoded at make_decision_client_node.hpp:103-110, so
  // leaving a param file entry out changes nothing.
  if (!node->has_parameter(behavior_name_ + ".box_size")) {
    node->declare_parameter(behavior_name_ + ".box_size", robot_params_.box_size);
  }
  node->get_parameter(behavior_name_ + ".box_size", robot_params_.box_size);

  if (!node->has_parameter(behavior_name_ + ".robot_base_offset_x")) {
    node->declare_parameter(behavior_name_ + ".robot_base_offset_x", robot_params_.robot_base_offset_x);
  }
  node->get_parameter(behavior_name_ + ".robot_base_offset_x", robot_params_.robot_base_offset_x);

  if (!node->has_parameter(behavior_name_ + ".link_l1")) {
    node->declare_parameter(behavior_name_ + ".link_l1", robot_params_.link_l1);
  }
  node->get_parameter(behavior_name_ + ".link_l1", robot_params_.link_l1);

  if (!node->has_parameter(behavior_name_ + ".link_l2")) {
    node->declare_parameter(behavior_name_ + ".link_l2", robot_params_.link_l2);
  }
  node->get_parameter(behavior_name_ + ".link_l2", robot_params_.link_l2);

  if (!node->has_parameter(behavior_name_ + ".gap_eef_close")) {
    node->declare_parameter(behavior_name_ + ".gap_eef_close", robot_params_.gap_eef_close);
  }
  node->get_parameter(behavior_name_ + ".gap_eef_close", robot_params_.gap_eef_close);

  if (!node->has_parameter(behavior_name_ + ".gap_eef_open")) {
    node->declare_parameter(behavior_name_ + ".gap_eef_open", robot_params_.gap_eef_open);
  }
  node->get_parameter(behavior_name_ + ".gap_eef_open", robot_params_.gap_eef_open);

  if (!node->has_parameter(behavior_name_ + ".limit_l1_down")) {
    node->declare_parameter(behavior_name_ + ".limit_l1_down", robot_params_.limit_l1_down);
  }
  node->get_parameter(behavior_name_ + ".limit_l1_down", robot_params_.limit_l1_down);

  if (!node->has_parameter(behavior_name_ + ".limit_l1_up")) {
    node->declare_parameter(behavior_name_ + ".limit_l1_up", robot_params_.limit_l1_up);
  }
  node->get_parameter(behavior_name_ + ".limit_l1_up", robot_params_.limit_l1_up);

  if (!node->has_parameter(behavior_name_ + ".state_file")) {
    node->declare_parameter(
      behavior_name_ + ".state_file", std::string("/tmp/bizon_<player_side>_last_moves.txt"));
  }
  node->get_parameter(behavior_name_ + ".state_file", state_file_template_);

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] DecisionPlugin configured (white: depth=%d skill=%d multipv=%d; "
    "black: depth=%d skill=%d multipv=%d; search_timeout=%.1fs)",
    behavior_name_.c_str(), white_strength_.search_depth, white_strength_.skill_level,
    white_strength_.multipv, black_strength_.search_depth, black_strength_.skill_level,
    black_strength_.multipv, search_timeout_s);
}

void DecisionPlugin::onCleanup()
{
  engine_.stop();
}

bool DecisionPlugin::ensure_engine_running()
{
  if (engine_.isRunning()) {
    return true;
  }
  if (!engine_.start(std::chrono::seconds(5))) {
    return false;
  }
  // On Jetson keep threads at 1 and hash small so the engine does not
  // contend with TensorRT running BoardPlugin's inference at the same time.
  engine_.setOption("Threads", std::to_string(engine_threads_));
  engine_.setOption("Hash", std::to_string(engine_hash_mb_));
  return true;
}

ResultStatus DecisionPlugin::onRun(const std::shared_ptr<const DecisionAction::Goal> command)
{
  auto node = node_.lock();

  if (command->player_side.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] empty player_side in goal", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 1};
  }
  if (command->fen.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] empty fen in goal", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 2};
  }

  player_side_ = command->player_side;
  opponent_side_ = (player_side_ == "white") ? "black" : "white";
  // Drives bizon_chess::mirrorForRobotSide()/squareToWorldMirrored() -- see
  // mirror_xy() and box_to_world_mirrored() below, which now just forward to
  // those. Set once per goal since player_side_ is itself per-goal.
  robot_params_.mirrored = (player_side_ != "white");
  fen_from_camera_ = command->fen;

  state_file_ = state_file_template_;
  const std::string placeholder = "<player_side>";
  const size_t pos = state_file_.find(placeholder);
  if (pos != std::string::npos) {
    state_file_.replace(pos, placeholder.size(), player_side_);
  }

  if (!std::filesystem::exists(state_file_)) {
    write_to_text_file("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
    RCLCPP_INFO(
      node->get_logger(), "[%s] created move-history file at: %s", behavior_name_.c_str(),
      state_file_.c_str());
  }

  if (!ensure_engine_running()) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] failed to start Stockfish: %s", behavior_name_.c_str(),
      engine_.lastError().c_str());
    return ResultStatus{Status::FAILED, 4};
  }

  return ResultStatus{Status::SUCCEEDED, 0};
}

ResultStatus DecisionPlugin::onCycleUpdate()
{
  auto node = node_.lock();
  reset_values();

  if (count_saved_promotion_pieces_ < 2) {
    return handle_save_piece();
  }

  move_owner_detected_ = who_is_owner_of_move();

  if (move_owner_detected_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[%s] failed to detect move owner", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 5};
  }

  if (move_owner_detected_ != player_side_) {
    // Invariant: waiting for the opponent is the normal steady state, not a
    // fault -- it returns SUCCESS with move_type "wait", and the tree's
    // MoveOrWaitForOpponent fallback skips the move subtree on that value.
    // Returning FAILED here would exhaust RecoveryNode's retries and
    // abandon the game mid-play.
    move_type_ = "wait";
    move_count_ = 0;
    RCLCPP_INFO(
      node->get_logger(), "[%s] opponent's move (%s); waiting", behavior_name_.c_str(),
      move_owner_detected_.c_str());
    return ResultStatus{Status::SUCCEEDED, 0};
  }

  fen_validated_ = get_last_move_from_text();

  if (is_checkmate(fen_validated_)) {
    return handle_checkmate(fen_validated_);
  }

  RCLCPP_INFO(
    node->get_logger(), "[%s] self move detected (%s); move will be made", behavior_name_.c_str(),
    move_owner_detected_.c_str());

  return handle_move();
}

ResultStatus DecisionPlugin::handle_save_piece()
{
  auto node = node_.lock();

  std::string box_from, box_to;
  if (count_saved_promotion_pieces_ == 0) {
    box_from = (player_side_ == "white") ? "a4" : "h5";
    box_to = (player_side_ == "white") ? "a1" : "h8";
  } else {
    box_from = (player_side_ == "white") ? "b4" : "g5";
    box_to = (player_side_ == "white") ? "a2" : "h7";
  }

  double x_from = 0.0, y_from = 0.0, x_to = 0.0, y_to = 0.0;
  if (!box_to_world_mirrored(box_from, x_from, y_from) ||
    !box_to_world_mirrored(box_to, x_to, y_to))
  {
    RCLCPP_ERROR(node->get_logger(), "[%s] malformed save-piece box", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 3};
  }

  // Offset applied after mirroring, matching the pre-refactor order exactly.
  y_to += robot_params_.box_size * 2;
  count_saved_promotion_pieces_++;

  if (!joint_targets_from_xy(x_from, y_from, move_from1_, move_from_down1_) ||
    !joint_targets_from_xy(x_to, y_to, move_to1_, move_to_down1_))
  {
    RCLCPP_ERROR(node->get_logger(), "[%s] unreachable save-piece target", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 3};
  }

  hand_close_position_ = {
    robot_params_.gap_eef_close, robot_params_.gap_eef_close, robot_params_.gap_eef_close};
  hand_open_position_ = {
    robot_params_.gap_eef_open, robot_params_.gap_eef_open, robot_params_.gap_eef_open};
  move_type_ = "save";
  move_count_ = 1;
  return ResultStatus{Status::SUCCEEDED, 0};
}

ResultStatus DecisionPlugin::handle_checkmate(const std::string & fen)
{
  auto node = node_.lock();
  const std::string box_killing_king = king_square(fen);

  double x_from = 0.0, y_from = 0.0;
  if (!box_to_world_mirrored(box_killing_king, x_from, y_from)) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] malformed king square '%s'", behavior_name_.c_str(),
      box_killing_king.c_str());
    return ResultStatus{Status::FAILED, 3};
  }
  double x_to = 0.0, y_to = 0.0;
  mirror_xy(x_to, y_to);

  if (!joint_targets_from_xy(x_from, y_from, move_from1_, move_from_down1_) ||
    !joint_targets_from_xy(x_to, y_to, move_to1_, move_to_down1_))
  {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] unreachable king-removal target", behavior_name_.c_str());
    return ResultStatus{Status::FAILED, 3};
  }

  move_from2_ = move_from_down2_ = move_to2_ = move_to_down2_ = std::vector<double>{0.0, 0.0, 0.0, 0.0};
  move_from3_ = move_from_down3_ = move_to3_ = move_to_down3_ = std::vector<double>{0.0, 0.0, 0.0, 0.0};

  hand_close_position_ = {
    robot_params_.gap_eef_close, robot_params_.gap_eef_close, robot_params_.gap_eef_close};
  hand_open_position_ = {
    robot_params_.gap_eef_open, robot_params_.gap_eef_open, robot_params_.gap_eef_open};
  move_type_ = "killking";
  move_count_ = 2;

  // At this point move_owner_detected_ == player_side_ (checked by the
  // caller), so fen -- which has no legal moves for the side to move -- is
  // checkmate delivered by whoever moved last: the opponent.
  RCLCPP_INFO(
    node->get_logger(), "[%s] checkmate in FEN %s; %s's king removed from %s",
    behavior_name_.c_str(), fen.c_str(), opponent_side_.c_str(), box_killing_king.c_str());
  return ResultStatus{Status::SUCCEEDED, 0};
}

ResultStatus DecisionPlugin::handle_move()
{
  auto node = node_.lock();

  if (!get_best_move(fen_validated_, move_best_)) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] failed to get best move from Stockfish: %s",
      behavior_name_.c_str(), engine_.lastError().c_str());
    // Force a clean restart on the next goal rather than risk reusing an
    // engine that just failed to answer within its deadline.
    engine_.stop();
    return ResultStatus{Status::FAILED, 6};
  }
  if (move_best_.size() == 5) {
    move_best_[4] = 'q';  // default promotion to queen; move type control below is unaffected
  }

  move_type_ = bizon_chess::classifyMoveType(fen_validated_, move_best_);
  RCLCPP_INFO(
    node->get_logger(), "[%s] best move: %s, move type: %s", behavior_name_.c_str(),
    move_best_.c_str(), move_type_.c_str());

  bool geometry_ok = true;
  if (move_type_ == "straight") {
    geometry_ok = handle_straight();
  } else if (
    move_type_ == "long_castle" || move_type_ == "short_castle" || move_type_ == "en_passant" ||
    move_type_ == "capture" || move_type_ == "promotion")
  {
    geometry_ok = handle_two_piece_move();
  } else if (move_type_ == "promotion_capture") {
    geometry_ok = handle_promotion_capture();
  }

  if (!geometry_ok) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] unreachable target for move %s", behavior_name_.c_str(),
      move_best_.c_str());
    return ResultStatus{Status::FAILED, 3};
  }

  std::string fen_desired;
  if (!apply_move_to_fen(fen_validated_, move_best_, fen_desired)) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] failed to apply move %s to fen", behavior_name_.c_str(),
      move_best_.c_str());
    engine_.stop();
    return ResultStatus{Status::FAILED, 6};
  }

  // Invariant: hold the expected post-move position, don't write it. Nothing
  // else writes our own move into the history file -- see the comment on
  // fen_pending_ in the header for why writing it here would deadlock the
  // game the first time the arm failed to actually make the move.
  fen_pending_ = fen_desired;

  hand_close_position_ = {
    robot_params_.gap_eef_close, robot_params_.gap_eef_close, robot_params_.gap_eef_close};
  hand_open_position_ = {
    robot_params_.gap_eef_open, robot_params_.gap_eef_open, robot_params_.gap_eef_open};

  if (move_type_ == "straight") {
    move_count_ = 1;
  } else if (move_type_ == "promotion_capture") {
    move_count_ = 3;
  } else {
    move_count_ = 2;
  }

  return ResultStatus{Status::SUCCEEDED, 0};
}

bool DecisionPlugin::handle_straight()
{
  const std::string box_from = move_best_.substr(0, 2);
  const std::string box_to = move_best_.substr(2, 2);

  double x_from = 0.0, y_from = 0.0, x_to = 0.0, y_to = 0.0;
  if (!box_to_world_mirrored(box_from, x_from, y_from) ||
    !box_to_world_mirrored(box_to, x_to, y_to))
  {
    return false;
  }
  box_from1_ = box_from;
  box_to1_ = box_to;
  return joint_targets_from_xy(x_from, y_from, move_from1_, move_from_down1_) &&
    joint_targets_from_xy(x_to, y_to, move_to1_, move_to_down1_);
}

bool DecisionPlugin::handle_two_piece_move()
{
  std::string box_from1 = move_best_.substr(0, 2);
  std::string box_to1 = move_best_.substr(2, 2);
  std::string box_from2, box_to2;
  int dv = 0, md = 0;

  const std::string board_only = fen_validated_.substr(0, fen_validated_.find(' '));

  if (move_type_ == "long_castle") {
    box_from2 = (player_side_ == "white") ? "a1" : "a8";
    box_to2 = (player_side_ == "white") ? "d1" : "d8";
    move_type_ = "castle";
  } else if (move_type_ == "short_castle") {
    box_from2 = (player_side_ == "white") ? "h1" : "h8";
    box_to2 = (player_side_ == "white") ? "f1" : "f8";
    move_type_ = "castle";
  } else if (move_type_ == "en_passant") {
    box_from2 = move_best_.substr(2, 2);
    box_from2[1] = static_cast<char>((player_side_ == "white") ? box_from2[1] - 1 : box_from2[1] + 1);

    get_captured_pieces_count(board_only, count_white_captured_, count_black_captured_);
    dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
    md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;
    box_to2 = (player_side_ == "white") ? ("h" + std::to_string(md)) : ("a" + std::to_string(9 - md));

    RCLCPP_WARN(
      node_.lock()->get_logger(),
      "[%s] en passant move detected; captured-piece box %s (unverified: no check that this square "
      "actually holds an opponent pawn)",
      behavior_name_.c_str(), box_from2.c_str());
  } else if (move_type_ == "capture") {
    box_from1 = move_best_.substr(2, 2);
    box_from2 = move_best_.substr(0, 2);
    box_to2 = move_best_.substr(2, 2);

    get_captured_pieces_count(board_only, count_white_captured_, count_black_captured_);
    dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
    md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;
    box_to1 = (player_side_ == "white") ? ("h" + std::to_string(md)) : ("a" + std::to_string(9 - md));
  } else if (move_type_ == "promotion") {
    box_from1 = (player_side_ == "white") ?
      ("a" + std::to_string(count_promotion_ + 1)) :
      ("h" + std::to_string(8 - count_promotion_));
    box_to1 = move_best_.substr(2, 2);

    // dv/md are computed here for parity with the pre-refactor code, which
    // computed them unconditionally before branching; they are unused for
    // "promotion", exactly as before.
    get_captured_pieces_count(board_only, count_white_captured_, count_black_captured_);
    dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
    md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;
    (void)dv;
    (void)md;

    box_from2 = move_best_.substr(0, 2);
    box_to2 = box_from1;
  }

  double x_from1 = 0.0, y_from1 = 0.0, x_to1 = 0.0, y_to1 = 0.0;
  double x_from2 = 0.0, y_from2 = 0.0, x_to2 = 0.0, y_to2 = 0.0;
  if (!square_to_world(box_from1, x_from1, y_from1) || !square_to_world(box_to1, x_to1, y_to1) ||
    !square_to_world(box_from2, x_from2, y_from2) || !square_to_world(box_to2, x_to2, y_to2))
  {
    return false;
  }

  // These y-offsets are applied to *unmirrored* board-frame coordinates,
  // before the mirror below -- matching the pre-refactor order exactly, and
  // load-bearing for the black-side robot: mirroring flips the sign of y,
  // so applying the same offset after mirroring would send the piece the
  // wrong way on that side.
  if (move_type_ == "en_passant") {
    y_to2 += (player_side_ == "white") ? -(dv + 2) * robot_params_.box_size : (dv + 2) * robot_params_.box_size;
  }
  if (move_type_ == "capture") {
    y_to1 += (player_side_ == "white") ? -(dv + 2) * robot_params_.box_size : (dv + 2) * robot_params_.box_size;
  }
  if (move_type_ == "promotion") {
    y_from1 += (player_side_ == "white") ? robot_params_.box_size * 2 : -robot_params_.box_size * 2;
    y_to2 += (player_side_ == "white") ? robot_params_.box_size * 2 : -robot_params_.box_size * 2;
    count_promotion_++;
  }

  mirror_xy(x_from1, y_from1);
  mirror_xy(x_to1, y_to1);
  mirror_xy(x_from2, y_from2);
  mirror_xy(x_to2, y_to2);

  box_from1_ = box_from1;
  box_to1_ = box_to1;
  box_from2_ = box_from2;
  box_to2_ = box_to2;

  return joint_targets_from_xy(x_from1, y_from1, move_from1_, move_from_down1_) &&
    joint_targets_from_xy(x_to1, y_to1, move_to1_, move_to_down1_) &&
    joint_targets_from_xy(x_from2, y_from2, move_from2_, move_from_down2_) &&
    joint_targets_from_xy(x_to2, y_to2, move_to2_, move_to_down2_);
}

bool DecisionPlugin::handle_promotion_capture()
{
  const std::string board_only = fen_validated_.substr(0, fen_validated_.find(' '));
  get_captured_pieces_count(board_only, count_white_captured_, count_black_captured_);
  const int dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
  const int md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;

  const std::string box_from1 = move_best_.substr(2, 2);
  const std::string box_to1 =
    (player_side_ == "white") ? ("h" + std::to_string(md)) : ("a" + std::to_string(9 - md));

  const std::string box_from2 = (player_side_ == "white") ?
    ("a" + std::to_string(count_promotion_ + 1)) :
    ("h" + std::to_string(8 - count_promotion_));
  const std::string box_to2 = move_best_.substr(2, 2);

  const std::string box_from3 = move_best_.substr(0, 2);
  const std::string box_to3 = box_from2;

  double x_from1 = 0.0, y_from1 = 0.0, x_to1 = 0.0, y_to1 = 0.0;
  double x_from2 = 0.0, y_from2 = 0.0, x_to2 = 0.0, y_to2 = 0.0;
  double x_from3 = 0.0, y_from3 = 0.0, x_to3 = 0.0, y_to3 = 0.0;
  if (!square_to_world(box_from1, x_from1, y_from1) || !square_to_world(box_to1, x_to1, y_to1) ||
    !square_to_world(box_from2, x_from2, y_from2) || !square_to_world(box_to2, x_to2, y_to2) ||
    !square_to_world(box_from3, x_from3, y_from3) || !square_to_world(box_to3, x_to3, y_to3))
  {
    return false;
  }

  // Offset before mirroring -- see the comment in handle_two_piece_move().
  y_to1 += (player_side_ == "white") ? -(dv + 2) * robot_params_.box_size : (dv + 2) * robot_params_.box_size;

  mirror_xy(x_from1, y_from1);
  mirror_xy(x_to1, y_to1);
  mirror_xy(x_from2, y_from2);
  mirror_xy(x_to2, y_to2);
  mirror_xy(x_from3, y_from3);
  mirror_xy(x_to3, y_to3);

  // These two offsets are applied *after* mirroring, matching the
  // pre-refactor order exactly -- unlike the y_to1 offset above.
  y_from2 += robot_params_.box_size * 2;
  y_to3 += robot_params_.box_size * 2;
  count_promotion_++;

  box_from1_ = box_from1;
  box_to1_ = box_to1;
  box_from2_ = box_from2;
  box_to2_ = box_to2;
  box_from3_ = box_from3;
  box_to3_ = box_to3;

  return joint_targets_from_xy(x_from1, y_from1, move_from1_, move_from_down1_) &&
    joint_targets_from_xy(x_to1, y_to1, move_to1_, move_to_down1_) &&
    joint_targets_from_xy(x_from2, y_from2, move_from2_, move_from_down2_) &&
    joint_targets_from_xy(x_to2, y_to2, move_to2_, move_to_down2_) &&
    joint_targets_from_xy(x_from3, y_from3, move_from3_, move_from_down3_) &&
    joint_targets_from_xy(x_to3, y_to3, move_to3_, move_to_down3_);
}

bool DecisionPlugin::square_to_world(const std::string & box, double & x, double & y)
{
  return bizon_chess::squareToWorld(box, robot_params_, x, y);
}

void DecisionPlugin::mirror_xy(double & x, double & y) const
{
  // See Task 7 Ruling 1: this is now the single call site for the mirroring
  // arithmetic, in bizon_chess, shared with ArmPlugin's collision-object
  // placement rather than a second hand-copied version of it.
  bizon_chess::mirrorForRobotSide(robot_params_, x, y);
}

bool DecisionPlugin::box_to_world_mirrored(const std::string & box, double & x, double & y)
{
  return bizon_chess::squareToWorldMirrored(box, robot_params_, x, y);
}

bool DecisionPlugin::joint_targets_from_xy(
  double x, double y, std::vector<double> & up_out, std::vector<double> & down_out)
{
  double q0 = 0.0, q2 = 0.0;
  if (!bizon_chess::worldToJointAngles(x, y, robot_params_, q0, q2)) {
    return false;
  }
  up_out = {q0, robot_params_.limit_l1_up, q2, 0.0};
  down_out = {q0, robot_params_.limit_l1_down, q2, 0.0};
  return true;
}

bool DecisionPlugin::confirm_pending_against_camera()
{
  if (fen_pending_.empty()) {
    return false;
  }

  auto node = node_.lock();
  const std::string pending_board = fen_pending_.substr(0, fen_pending_.find(' '));

  // The board is exactly where our move should have left it: the move was
  // executed and the opponent has not replied yet.
  if (pending_board == fen_from_camera_) {
    write_to_text_file(fen_pending_);
    fen_from_text_ = fen_pending_;
    fen_from_text_only_board_ = pending_board;
    RCLCPP_INFO(
      node->get_logger(), "[%s] own move confirmed by camera; history advanced to: %s",
      behavior_name_.c_str(), fen_pending_.c_str());
    fen_pending_.clear();
    return true;
  }

  // Our move was executed and the opponent has already replied. Both plies
  // belong in the history, oldest first.
  for (const auto & reply_fen : possible_next_moves_from_valid_fen(fen_pending_)) {
    if (reply_fen.substr(0, reply_fen.find(' ')) == fen_from_camera_) {
      write_to_text_file(fen_pending_);
      write_to_text_file(reply_fen);
      fen_from_text_ = reply_fen;
      fen_from_text_only_board_ = fen_from_camera_;
      RCLCPP_INFO(
        node->get_logger(),
        "[%s] own move confirmed by camera and the opponent has replied; history advanced to: %s",
        behavior_name_.c_str(), reply_fen.c_str());
      fen_pending_.clear();
      return true;
    }
  }

  // The board does not show what we planned: the move was never executed,
  // it was executed wrongly, or perception is off. The camera is the
  // authority, so drop the expectation and let the ladder below reconcile
  // from the last position the file is sure about.
  RCLCPP_WARN(
    node->get_logger(),
    "[%s] pending move is not on the board (pending: %s, camera: %s); discarding it and "
    "reconciling from the history file",
    behavior_name_.c_str(), fen_pending_.c_str(), fen_from_camera_.c_str());
  fen_pending_.clear();
  return false;
}

std::string DecisionPlugin::who_is_owner_of_move()
{
  std::string owner;
  auto node = node_.lock();

  fen_from_text_ = get_last_move_from_text();
  fen_from_text_only_board_ = fen_from_text_.substr(0, fen_from_text_.find(' '));

  // Commit our own last move to the history first, if the camera backs it
  // up. Without this the file never advances on its own and every single
  // call has to rediscover the position through the search ladder below.
  confirm_pending_against_camera();

  RCLCPP_INFO(node->get_logger(), "[%s] camera FEN: %s", behavior_name_.c_str(), fen_from_camera_.c_str());
  RCLCPP_INFO(node->get_logger(), "[%s] text FEN:   %s", behavior_name_.c_str(), fen_from_text_.c_str());

  if (fen_from_camera_ == fen_from_text_only_board_) {
    // Owner detection reads the side-to-move field of the *matched*
    // position (fen_from_text_ itself), never off a parent it descends
    // from -- see bizon_chess::side_to_move() vs side_to_move_after_one_ply().
    owner = bizon_chess::side_to_move(fen_from_text_);
    return owner;
  }

  RCLCPP_WARN(
    node->get_logger(), "[%s] FENs do not match! Camera: %s, Text: %s", behavior_name_.c_str(),
    fen_from_camera_.c_str(), fen_from_text_only_board_.c_str());

  if (compare_fens_without_classes(fen_from_camera_, fen_from_text_only_board_)) {
    RCLCPP_WARN(
      node->get_logger(), "[%s] FENs match ignoring piece classes; possible OCR misclassification",
      behavior_name_.c_str());
    return bizon_chess::side_to_move(fen_from_text_);
  }

  const std::vector<std::string> possible_next_fens = possible_next_moves_from_valid_fen(fen_from_text_);

  // Ladder order matters: an exact match at any depth is stronger evidence
  // than a class-insensitive match at depth one. Trying depth-1 fuzzy first
  // let a single misclassified piece outrank a position the engine can
  // reproduce exactly.

  // First degree, exact.
  for (const auto & possible_fen : possible_next_fens) {
    if (possible_fen.substr(0, possible_fen.find(' ')) == fen_from_camera_) {
      write_to_text_file(possible_fen);
      owner = bizon_chess::side_to_move(possible_fen);
      RCLCPP_INFO(
        node->get_logger(), "[%s] owner from first-degree exact match: %s (fen: %s)",
        behavior_name_.c_str(), owner.c_str(), possible_fen.c_str());
      return owner;
    }
  }

  // Second degree, exact.
  for (const auto & possible_fen : possible_next_fens) {
    for (const auto & possible_fen_second : possible_next_moves_from_valid_fen(possible_fen)) {
      if (possible_fen_second.substr(0, possible_fen_second.find(' ')) == fen_from_camera_) {
        // possible_fen is the intermediate position the board passed
        // through; possible_fen_second is what the camera sees now. Both
        // go into the history, but the owner is read off the position
        // actually on the board -- never off a parent.
        write_to_text_file(possible_fen);
        write_to_text_file(possible_fen_second);
        owner = bizon_chess::side_to_move(possible_fen_second);
        RCLCPP_INFO(
          node->get_logger(), "[%s] owner from second-degree exact match: %s (fen: %s)",
          behavior_name_.c_str(), owner.c_str(), possible_fen_second.c_str());
        return owner;
      }
    }
  }

  // First degree, ignoring piece classes.
  for (const auto & possible_fen : possible_next_fens) {
    const std::string board_only = possible_fen.substr(0, possible_fen.find(' '));
    if (compare_fens_without_classes(fen_from_camera_, board_only)) {
      write_to_text_file(possible_fen);
      owner = bizon_chess::side_to_move(possible_fen);
      RCLCPP_WARN(
        node->get_logger(), "[%s] owner from first-degree class-insensitive match: %s (fen: %s)",
        behavior_name_.c_str(), owner.c_str(), possible_fen.c_str());
      return owner;
    }
  }

  // Second degree, ignoring piece classes.
  for (const auto & possible_fen : possible_next_fens) {
    for (const auto & possible_fen_second : possible_next_moves_from_valid_fen(possible_fen)) {
      const std::string board_only_second = possible_fen_second.substr(0, possible_fen_second.find(' '));
      if (compare_fens_without_classes(fen_from_camera_, board_only_second)) {
        write_to_text_file(possible_fen);
        write_to_text_file(possible_fen_second);
        owner = bizon_chess::side_to_move(possible_fen_second);
        RCLCPP_WARN(
          node->get_logger(),
          "[%s] owner from second-degree class-insensitive match: %s (fen: %s)",
          behavior_name_.c_str(), owner.c_str(), possible_fen_second.c_str());
        return owner;
      }
    }
  }

  RCLCPP_ERROR(
    node->get_logger(), "[%s] no match at any depth; cannot detect move owner", behavior_name_.c_str());
  return owner;
}

bool DecisionPlugin::apply_move_to_fen(
  const std::string & base_fen, const std::string & move, std::string & fen_out)
{
  return engine_.applyMove(base_fen, move, search_timeout_, fen_out);
}

std::vector<std::string> DecisionPlugin::possible_next_moves_from_valid_fen(const std::string & fen)
{
  std::vector<std::string> fens_possible;
  std::vector<std::string> moves;
  if (!engine_.legalMoves(fen, search_timeout_, moves)) {
    RCLCPP_ERROR(
      node_.lock()->get_logger(), "[%s] legalMoves failed: %s", behavior_name_.c_str(),
      engine_.lastError().c_str());
    // This is the reconciliation ladder that invariant (a)'s
    // confirm_pending_against_camera() and who_is_owner_of_move() both run
    // on. A stuck-but-still-alive engine would otherwise keep failing here
    // forever: isRunning() only reports whether the process exists, not
    // whether it is answering, so ensure_engine_running() would never
    // restart it on its own. Force the restart explicitly, the same way
    // handle_move() already does on a bestMove()/applyMove() failure.
    engine_.stop();
    return fens_possible;
  }

  fens_possible.reserve(moves.size());
  for (const auto & move : moves) {
    std::string fen_possible;
    if (apply_move_to_fen(fen, move, fen_possible)) {
      fens_possible.push_back(fen_possible);
    } else {
      RCLCPP_ERROR(
        node_.lock()->get_logger(), "[%s] failed to get possible FEN for move: %s",
        behavior_name_.c_str(), move.c_str());
      // Same reasoning as above; a fresh engine on the next call is safer
      // than one that just proved unreliable mid-ladder.
      engine_.stop();
    }
  }
  return fens_possible;
}

bool DecisionPlugin::is_checkmate(const std::string & fen)
{
  std::vector<std::string> moves;
  if (!engine_.legalMoves(fen, search_timeout_, moves)) {
    RCLCPP_ERROR(
      node_.lock()->get_logger(), "[%s] legalMoves failed while checking for checkmate: %s",
      behavior_name_.c_str(), engine_.lastError().c_str());
    // See the comment in possible_next_moves_from_valid_fen(): force a
    // restart rather than let a wedged engine fail identically forever.
    engine_.stop();
    return false;
  }
  return moves.empty();
}

bool DecisionPlugin::get_best_move(const std::string & fen, std::string & move_out)
{
  // Selected by player_side_ rather than a hardcoded branch -- see the
  // white_strength_/black_strength_ comment in the header for why the two
  // sides are allowed to differ at all. Sent on every search, not once at
  // startup: a single long-lived engine process can be asked to move for
  // either side across goals, so a value set once at start() could leak
  // from one side's search into the other's.
  const EngineStrength & strength = (player_side_ == "white") ? white_strength_ : black_strength_;
  engine_.setOption("Skill Level", std::to_string(strength.skill_level));
  engine_.setOption("MultiPV", std::to_string(strength.multipv));

  return engine_.bestMove(fen, strength.search_depth, search_timeout_, move_out);
}

std::string DecisionPlugin::get_last_move_from_text()
{
  std::ifstream file(state_file_);
  std::string line, last;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      last = line;
    }
  }
  return last;
}

bool DecisionPlugin::write_to_text_file(const std::string & fen)
{
  {
    std::ifstream in(state_file_, std::ios::ate | std::ios::binary);
    if (in.is_open() && in.tellg() > 0) {
      in.seekg(-1, std::ios::end);
      char last_char;
      in.get(last_char);
      if (last_char != '\n') {
        std::ofstream fix(state_file_, std::ios::app);
        fix << '\n';
      }
    }
  }

  std::ofstream file(state_file_, std::ios::app);
  if (!file.is_open()) {
    RCLCPP_ERROR(
      node_.lock()->get_logger(), "[%s] failed to open state file for writing: %s",
      behavior_name_.c_str(), state_file_.c_str());
    return false;
  }
  file << fen << std::endl;
  return true;
}

void DecisionPlugin::reset_values()
{
  fen_from_text_.clear();
  fen_from_text_only_board_.clear();
  fen_validated_.clear();
  move_owner_detected_.clear();
  move_best_.clear();
  move_type_.clear();
  move_count_ = 0;
  count_white_captured_ = 0;
  count_black_captured_ = 0;
  box_from1_.clear();
  box_to1_.clear();
  box_from2_.clear();
  box_to2_.clear();
  box_from3_.clear();
  box_to3_.clear();
}

void DecisionPlugin::onActionCompletion(std::shared_ptr<DecisionAction::Result> result)
{
  result->move_type = move_type_;
  result->move_count = move_count_;
  result->hand_open_position = hand_open_position_;
  result->hand_close_position = hand_close_position_;
  result->move_from1 = move_from1_;
  result->move_from_down1 = move_from_down1_;
  result->move_to1 = move_to1_;
  result->move_to_down1 = move_to_down1_;
  result->move_from2 = move_from2_;
  result->move_from_down2 = move_from_down2_;
  result->move_to2 = move_to2_;
  result->move_to_down2 = move_to_down2_;
  result->move_from3 = move_from3_;
  result->move_from_down3 = move_from_down3_;
  result->move_to3 = move_to3_;
  result->move_to_down3 = move_to_down3_;
  result->box_from1 = box_from1_;
  result->box_to1 = box_to1_;
  result->box_from2 = box_from2_;
  result->box_to2 = box_to2_;
  result->box_from3 = box_from3_;
  result->box_to3 = box_to3_;
}

}  // namespace bizon_behaviors

PLUGINLIB_EXPORT_CLASS(bizon_behaviors::DecisionPlugin, bizon_core::Behavior)
