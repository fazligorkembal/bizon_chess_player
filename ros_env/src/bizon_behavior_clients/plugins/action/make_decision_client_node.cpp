#include "bizon_behavior_clients/plugins/action/make_decision_client_node.hpp"
#include "bizon_behavior_clients/plugins/action/fen_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <vector>

namespace bizon_behavior_clients
{
    inline bool compare_fens_without_classes(const std::string &fen_camera, const std::string &fen_text)
    {
        auto replace_letters = [](const std::string &fen)
        {
            std::string result;
            for (char c : fen)
            {
                if (std::isupper(c))
                    result += 'X';
                else if (std::islower(c))
                    result += 'x';
                else
                    result += c;
            }
            return result;
        };
        // DEBUG, not ERROR: this runs once per candidate inside the recovery
        // ladder's search loops, so at ERROR it floods the console with what is
        // ordinary search progress and buries the real failures.
        const std::string masked_camera = replace_letters(fen_camera);
        const std::string masked_text = replace_letters(fen_text);
        RCLCPP_DEBUG(rclcpp::get_logger("MakeDecisionNode"),
                     "Comparing FENs without classes. Camera FEN: %s, Text FEN: %s, After replacement Camera FEN: %s, After replacement Text FEN: %s",
                     fen_camera.c_str(), fen_text.c_str(), masked_camera.c_str(), masked_text.c_str());
        return masked_camera == masked_text;
    }

    inline std::string get_move_owner_from_text(const std::string &fen)
    {
        size_t space_pos = fen.find(' ');
        if (space_pos != std::string::npos && space_pos + 1 < fen.size())
            return fen.substr(space_pos + 1, 1);
        return "";
    }

    inline std::string get_king_square(const std::string &fen)
    {
        std::string side = get_move_owner_from_text(fen);
        char king = (side == "w") ? 'K' : 'k';
        std::string board = fen.substr(0, fen.find(' '));

        int rank = 8;
        int file = 0;
        for (char c : board)
        {
            if (c == '/')
            {
                rank--;
                file = 0;
            }
            else if (c >= '1' && c <= '8')
            {
                file += c - '0';
            }
            else
            {
                if (c == king)
                    return std::string(1, 'a' + file) + std::to_string(rank);
                file++;
            }
        }
        return "";
    }

    inline void get_box_location_wrt_world(const std::string &box_name, const float box_size, float &x, float &y)
    {
        int col = box_name[0] - 'a'; // 0 (a) -> 7 (h)
        int row = box_name[1] - '1'; // 0 (1) -> 7 (8)

        x = -3.5f * box_size + row * box_size;
        y = 3.5f * box_size - col * box_size;
    }

    inline void calculate_joint_angles(float x, float y, float l1, float l2, float &q1, float &q2)
    {
        float D = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        q2 = atan2f(sqrtf(1 - D * D), D);
        q1 = atan2f(y, x) - atan2f(l2 * sinf(q2), l1 + l2 * cosf(q2));
    }

    inline void get_captured_pieces_count(const std::string &fen, int &white_captured_count, int &black_captured_count)
    {
        white_captured_count = black_captured_count = 0;
        for (char c : fen)
        {
            if (c >= 'A' && c <= 'Z')
                white_captured_count++;
            else if (c >= 'a' && c <= 'z')
                black_captured_count++;
        }
        white_captured_count = 16 - white_captured_count;
        black_captured_count = 16 - black_captured_count;
    }

    MakeDecisionNode::MakeDecisionNode(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"), "MakeDecisionNode created");

        if (!getInput("player_side", player_side_) || player_side_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("MakeDecisionNode"),
                         "Failed to get player_side input or player_side is empty");
            throw std::runtime_error("player_side input is required and cannot be empty");
        }

        last_moves_path += "/" + player_side_ + "_last_moves.txt";
        opponent_side_ = (player_side_ == "white") ? "black" : "white";

        if (!std::filesystem::exists(last_moves_path))
        {
            write_to_text_file("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                        "Created last moves file at: %s", last_moves_path.c_str());
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                        "Last moves file already exists at: %s", last_moves_path.c_str());
        }

        pipe(to_sf_);
        pipe(from_sf_);

        pid_ = fork();

        if (pid_ == 0)
        {
            close(to_sf_[1]);
            close(from_sf_[0]);

            dup2(to_sf_[0], STDIN_FILENO);
            dup2(from_sf_[1], STDOUT_FILENO);

            int devnull = open("/dev/null", O_WRONLY);
            dup2(devnull, STDERR_FILENO);
            close(devnull);

            close(to_sf_[0]);
            close(from_sf_[1]);

            execlp("stockfish", "stockfish", nullptr);
            perror("execlp failed");
            exit(1);
        }

        close(to_sf_[0]);
        close(from_sf_[1]);

        send_command("uci\n");
        read_until("uciok");

        send_command("isready\n");
        read_until("readyok");

        RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                    "Stockfish ready");
    }

    BT::NodeStatus MakeDecisionNode::tick()
    {
        reset_values();

        if (count_saved_promotion_pieces_ < 2)
        {
            move_type_ = "save";
            setOutput("move_type", move_type_);
            setOutput("move_count", 1);

            if (count_saved_promotion_pieces_ == 0)
            {
                box_from_1_ = (player_side_ == "white") ? "a4" : "h5";
                box_to_1_ = (player_side_ == "white") ? "a1" : "h8";
            }
            else if (count_saved_promotion_pieces_ == 1)
            {
                box_from_1_ = (player_side_ == "white") ? "b4" : "g5";
                box_to_1_ = (player_side_ == "white") ? "a2" : "h7";
            }

            get_box_location_wrt_world(box_from_1_, box_size, x_from_1_, y_from_1_);
            get_box_location_wrt_world(box_to_1_, box_size, x_to_1_, y_to_1_);

            if (player_side_ == "white")
            {
                x_from_1_ -= robot_position_wrt_world_x_;
                x_to_1_ -= robot_position_wrt_world_x_;
            }
            else
            {
                x_from_1_ = -x_from_1_ - robot_position_wrt_world_x_;
                y_from_1_ = -y_from_1_;
                x_to_1_ = -x_to_1_ - robot_position_wrt_world_x_;
                y_to_1_ = -y_to_1_;
            }

            y_to_1_ += box_size * 2;
            count_saved_promotion_pieces_++;

            calculate_joint_angles(x_from_1_, y_from_1_, length_l1_, length_l2_, q0_from_1_, q2_from_1_);
            calculate_joint_angles(x_to_1_, y_to_1_, length_l1_, length_l2_, q0_to_1_, q2_to_1_);

            setOutput("hand_close_position", std::vector<double>{gap_eef_close_, gap_eef_close_, gap_eef_close_});
            setOutput("hand_open_position", std::vector<double>{gap_eef_open_, gap_eef_open_, gap_eef_open_});
            setOutput("move_from1", std::vector<double>{q0_from_1_, limit_l1_up_, q2_from_1_, q3_from_1_});
            setOutput("move_from_down1", std::vector<double>{q0_from_1_, limit_l1_down_, q2_from_1_, q3_from_1_});
            setOutput("move_to1", std::vector<double>{q0_to_1_, limit_l1_up_, q2_to_1_, q3_to_1_});
            setOutput("move_to_down1", std::vector<double>{q0_to_1_, limit_l1_down_, q2_to_1_, q3_to_1_});

            return BT::NodeStatus::SUCCESS;
        }

        move_owner_detected_ = who_is_owner_of_move();

        if (move_owner_detected_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("MakeDecisionNode"), "Failed to detect move owner. Retrying...");
            return BT::NodeStatus::FAILURE;
        }
        else if (move_owner_detected_ != player_side_)
        {
            // It is the opponent's turn. That is the normal steady state, not a
            // fault: against a human it can hold for minutes. Reporting FAILURE
            // here put the fault inside RecoveryNode's work branch, so three
            // waiting ticks exhausted the retries and the tree gave up mid-game.
            // Report success with move_type "wait" instead; the tree's
            // MoveOrWaitForOpponent guard skips the move subtree, the loop comes
            // back round, and the board is read again.
            move_type_ = "wait";
            setOutput("move_type", move_type_);
            setOutput("move_count", 0);
            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                        "Opponent move detected. Current owner: %s. Waiting", move_owner_detected_.c_str());
            return BT::NodeStatus::SUCCESS;
        }

        fen_validated_ = get_last_move_from_text();
        if (is_checkmate(fen_validated_))
        {
            std::string winner = (move_owner_detected_ == player_side_) ? opponent_side_ : player_side_;
            std::string loser = (move_owner_detected_ == player_side_) ? player_side_ : opponent_side_;

            std::string box_killing_king = get_king_square(fen_validated_);

            box_from_1_ = box_killing_king;
            get_box_location_wrt_world(box_from_1_, box_size, x_from_1_, y_from_1_);

            x_to_1_ = 0.0;
            y_to_1_ = 0.0;

            if (player_side_ == "white")
            {
                x_from_1_ -= robot_position_wrt_world_x_;
                x_to_1_ -= robot_position_wrt_world_x_;
            }
            else
            {
                x_from_1_ = -x_from_1_ - robot_position_wrt_world_x_;
                y_from_1_ = -y_from_1_;
                x_to_1_ = -x_to_1_ - robot_position_wrt_world_x_;
                y_to_1_ = -y_to_1_;
            }

            calculate_joint_angles(x_from_1_, y_from_1_, length_l1_, length_l2_, q0_from_1_, q2_from_1_);
            calculate_joint_angles(x_to_1_, y_to_1_, length_l1_, length_l2_, q0_to_1_, q2_to_1_);

            RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                        "TODO: ADD FEN DESIRED INPUT AND OUTPUT TREE, AND CHECK IF THE MOVE APPLIED TO FEN VALIDATED GIVES THE DESIRED FEN. IF NOT, RETRY OR THROW ERROR. ALSO ADD CHECKMATE CONTROL AND MOVE TYPE CONTROL FOR OTHER MOVE TYPES (CASTLE, PROMOTION, EN PASSANT, CAPTURE, STRAIGHT). ALSO ADD OUTPUT PORTS FOR ALL THESE VALUES TO BE USED IN OTHER NODES. ALSO ADD MORE DETAILED LOGGING FOR EACH STEP.");
            setOutput("hand_close_position", std::vector<double>{gap_eef_close_, gap_eef_close_, gap_eef_close_});
            setOutput("hand_open_position", std::vector<double>{gap_eef_open_, gap_eef_open_, gap_eef_open_});
            setOutput("move_from1", std::vector<double>{q0_from_1_, limit_l1_up_, q2_from_1_, q3_from_1_});
            setOutput("move_from_down1", std::vector<double>{q0_from_1_, limit_l1_down_, q2_from_1_, q3_from_1_});
            setOutput("move_to1", std::vector<double>{q0_to_1_, limit_l1_up_, q2_to_1_, q3_to_1_});
            setOutput("move_to_down1", std::vector<double>{q0_to_1_, limit_l1_down_, q2_to_1_, q3_to_1_});
            setOutput("move_from2", std::vector<double>{0.0, 0.0, 0.0, 0.0});
            setOutput("move_from_down2", std::vector<double>{0.0, 0.0, 0.0, 0.0});
            setOutput("move_to2", std::vector<double>{0.0, 0.0, 0.0, 0.0});
            setOutput("move_to_down2", std::vector<double>{0.0, 0.0, 0.0, 0.0});
            setOutput("move_from3", std::vector<double>{0.0, 0.0, 0.0, 0.0});
            setOutput("move_from_down3", std::vector<double>{0.0, 0.0, 0.0, 0.0});
            setOutput("move_to3", std::vector<double>{0.0, 0.0, 0.0, 0.0});
            setOutput("move_to_down3", std::vector<double>{0.0, 0.0, 0.0, 0.0});

            setOutput("move_type", "killking");
            setOutput("move_count", 2);

            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                        "Checkmate detected in FEN: %s. Winner: %s. Opponent king is in box: %s. Game over.", fen_validated_.c_str(), winner.c_str(), box_killing_king.c_str());
            return BT::NodeStatus::SUCCESS;
        }

        RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                    "Self move detected. Current owner: %s. Move will be made", move_owner_detected_.c_str());

        move_best_ = get_best_move(fen_validated_);
        if(move_best_.size() == 5)
        {
            move_best_[4] = 'q'; // default promotion to queen, will be handled in move type control for promotion move type
        }

        if (move_best_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("MakeDecisionNode"), "Failed to get best move from Stockfish. Retrying...");
            return BT::NodeStatus::FAILURE;
        }

        move_type_ = get_move_type(fen_validated_, move_best_);

        RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                    "Best move: %s, Move type: %s", move_best_.c_str(), move_type_.c_str());

        if (move_type_ == "straight")
        {

            box_from_1_ = move_best_.substr(0, 2);
            box_to_1_ = move_best_.substr(2, 2);
            get_box_location_wrt_world(box_from_1_, box_size, x_from_1_, y_from_1_);
            get_box_location_wrt_world(box_to_1_, box_size, x_to_1_, y_to_1_);

            if (player_side_ == "white")
            {
                x_from_1_ -= robot_position_wrt_world_x_;
                x_to_1_ -= robot_position_wrt_world_x_;
            }
            else
            {
                x_from_1_ = -x_from_1_ - robot_position_wrt_world_x_;
                y_from_1_ = -y_from_1_;
                x_to_1_ = -x_to_1_ - robot_position_wrt_world_x_;
                y_to_1_ = -y_to_1_;
            }

            calculate_joint_angles(x_from_1_, y_from_1_, length_l1_, length_l2_, q0_from_1_, q2_from_1_);
            calculate_joint_angles(x_to_1_, y_to_1_, length_l1_, length_l2_, q0_to_1_, q2_to_1_);

            RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                        "TODO: ADD FEN DESIRED INPUT AND OUTPUT TREE, AND CHECK IF THE MOVE APPLIED TO FEN VALIDATED GIVES THE DESIRED FEN. IF NOT, RETRY OR THROW ERROR. ALSO ADD CHECKMATE CONTROL AND MOVE TYPE CONTROL FOR OTHER MOVE TYPES (CASTLE, PROMOTION, EN PASSANT, CAPTURE, STRAIGHT). ALSO ADD OUTPUT PORTS FOR ALL THESE VALUES TO BE USED IN OTHER NODES. ALSO ADD MORE DETAILED LOGGING FOR EACH STEP.");

            setOutput("move_from1", std::vector<double>{q0_from_1_, limit_l1_up_, q2_from_1_, q3_from_1_});
            setOutput("move_from_down1", std::vector<double>{q0_from_1_, limit_l1_down_, q2_from_1_, q3_from_1_});
            setOutput("move_to1", std::vector<double>{q0_to_1_, limit_l1_up_, q2_to_1_, q3_to_1_});
            setOutput("move_to_down1", std::vector<double>{q0_to_1_, limit_l1_down_, q2_to_1_, q3_to_1_});
        }
        else if (move_type_ == "long_castle" || move_type_ == "short_castle" || move_type_ == "en_passant" || move_type_ == "capture" || move_type_ == "promotion")
        {
            box_from_1_ = move_best_.substr(0, 2);
            box_to_1_ = move_best_.substr(2, 2);

            if (move_type_ == "long_castle")
            {
                box_from_2_ = (player_side_ == "white") ? "a1" : "a8";
                box_to_2_ = (player_side_ == "white") ? "d1" : "d8";
                move_type_ = "castle";
            }
            else if (move_type_ == "short_castle")
            {
                box_from_2_ = (player_side_ == "white") ? "h1" : "h8";
                box_to_2_ = (player_side_ == "white") ? "f1" : "f8";
                move_type_ = "castle";
            }
            else if (move_type_ == "en_passant")
            {
                box_from_2_ = move_best_.substr(2, 2);

                if (player_side_ == "white")
                    box_from_2_[1] = box_from_2_[1] - 1;
                else
                    box_from_2_[1] = box_from_2_[1] + 1;

                get_captured_pieces_count(fen_validated_.find(' ') != std::string::npos ? fen_validated_.substr(0, fen_validated_.find(' ')) : fen_validated_, count_white_captured_, count_black_captured_);

                dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
                md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;
                box_to_2_ = (player_side_ == "white") ? ("h" + std::to_string(md)) : ("a" + std::to_string(9 - md));

                RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                            "En passant move detected. Captured piece box: %s. TODO: ADD CONTROL TO CHECK IF THE PIECE IN THIS BOX IS A PAWN AND BELONGS TO OPPONENT BEFORE MAKING THE MOVE.", box_from_2_.c_str());
            }
            else if (move_type_ == "capture")
            {
                box_from_1_ = move_best_.substr(2, 2);
                // box_to_1_ = box_from_2_;

                box_from_2_ = move_best_.substr(0, 2);
                box_to_2_ = move_best_.substr(2, 2);

                get_captured_pieces_count(fen_validated_.find(' ') != std::string::npos ? fen_validated_.substr(0, fen_validated_.find(' ')) : fen_validated_, count_white_captured_, count_black_captured_);
                dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
                md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;
                box_to_1_ = (player_side_ == "white") ? ("h" + std::to_string(md)) : ("a" + std::to_string(9 - md));
            }else if (move_type_ == "promotion")
            {
                box_from_1_ = (player_side_ == "white") ? "a" + std::to_string(count_promotion_ + 1) : "h" + std::to_string(8 - count_promotion_);
                box_to_1_ = move_best_.substr(2, 2);

                get_captured_pieces_count(fen_validated_.find(' ') != std::string::npos ? fen_validated_.substr(0, fen_validated_.find(' ')) : fen_validated_, count_white_captured_, count_black_captured_);
                dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
                md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;

                box_from_2_ = move_best_.substr(0, 2);
                box_to_2_ = box_from_1_;
            }

            get_box_location_wrt_world(box_from_1_, box_size, x_from_1_, y_from_1_);
            get_box_location_wrt_world(box_to_1_, box_size, x_to_1_, y_to_1_);
            get_box_location_wrt_world(box_from_2_, box_size, x_from_2_, y_from_2_);
            get_box_location_wrt_world(box_to_2_, box_size, x_to_2_, y_to_2_);

            if (move_type_ == "en_passant")
            {
                y_to_2_ += (player_side_ == "white") ? -(dv + 2) * box_size : (dv + 2) * box_size;
            }

            if (move_type_ == "capture")
            {
                y_to_1_ += (player_side_ == "white") ? -(dv + 2) * box_size : (dv + 2) * box_size;
            }

            if (move_type_ == "promotion")
            {
                if (player_side_ == "white")
                {
                    y_from_1_ += box_size * 2;
                    y_to_2_ += box_size * 2;
                }
                else
                {
                    y_from_1_ -= box_size * 2;
                    y_to_2_ -= box_size * 2;
                }
                count_promotion_++;
            }

            if (player_side_ == "white")
            {
                x_from_1_ -= robot_position_wrt_world_x_;
                x_to_1_ -= robot_position_wrt_world_x_;
                x_from_2_ -= robot_position_wrt_world_x_;
                x_to_2_ -= robot_position_wrt_world_x_;
            }
            else
            {
                x_from_1_ = -x_from_1_ - robot_position_wrt_world_x_;
                y_from_1_ = -y_from_1_;
                x_to_1_ = -x_to_1_ - robot_position_wrt_world_x_;
                y_to_1_ = -y_to_1_;
                x_from_2_ = -x_from_2_ - robot_position_wrt_world_x_;
                y_from_2_ = -y_from_2_;
                x_to_2_ = -x_to_2_ - robot_position_wrt_world_x_;
                y_to_2_ = -y_to_2_;
            }

            calculate_joint_angles(x_from_1_, y_from_1_, length_l1_, length_l2_, q0_from_1_, q2_from_1_);
            calculate_joint_angles(x_to_1_, y_to_1_, length_l1_, length_l2_, q0_to_1_, q2_to_1_);
            calculate_joint_angles(x_from_2_, y_from_2_, length_l1_, length_l2_, q0_from_2_, q2_from_2_);
            calculate_joint_angles(x_to_2_, y_to_2_, length_l1_, length_l2_, q0_to_2_, q2_to_2_);

            setOutput("move_from1", std::vector<double>{q0_from_1_, limit_l1_up_, q2_from_1_, q3_from_1_});
            setOutput("move_from_down1", std::vector<double>{q0_from_1_, limit_l1_down_, q2_from_1_, q3_from_1_});
            setOutput("move_to1", std::vector<double>{q0_to_1_, limit_l1_up_, q2_to_1_, q3_to_1_});
            setOutput("move_to_down1", std::vector<double>{q0_to_1_, limit_l1_down_, q2_to_1_, q3_to_1_});
            setOutput("move_from2", std::vector<double>{q0_from_2_, limit_l1_up_, q2_from_2_, q3_from_2_});
            setOutput("move_from_down2", std::vector<double>{q0_from_2_, limit_l1_down_, q2_from_2_, q3_from_2_});
            setOutput("move_to_down2", std::vector<double>{q0_to_2_, limit_l1_down_, q2_to_2_, q3_to_2_});
            setOutput("move_to2", std::vector<double>{q0_to_2_, limit_l1_up_, q2_to_2_, q3_to_2_});
        }
        else if (move_type_ == "promotion_capture")
        {
            get_captured_pieces_count(fen_validated_.find(' ') != std::string::npos ? fen_validated_.substr(0, fen_validated_.find(' ')) : fen_validated_, count_white_captured_, count_black_captured_);
            dv = (player_side_ == "white") ? count_black_captured_ / 4 + 1 : count_white_captured_ / 4 + 1;
            md = (player_side_ == "white") ? count_black_captured_ % 4 + 1 : count_white_captured_ % 4 + 1;

            box_from_1_ = move_best_.substr(2, 2);
            box_to_1_ = (player_side_ == "white") ? ("h" + std::to_string(md)) : ("a" + std::to_string(9 - md));

            
            box_from_2_ = (player_side_ == "white") ? "a" + std::to_string(count_promotion_ + 1) : "h" + std::to_string(8 - count_promotion_);
            box_to_2_ = move_best_.substr(2, 2);
            
            box_from_3_ = move_best_.substr(0, 2);
            box_to_3_ = box_from_2_;


            get_box_location_wrt_world(box_from_1_, box_size, x_from_1_, y_from_1_);
            get_box_location_wrt_world(box_to_1_, box_size, x_to_1_, y_to_1_);
            get_box_location_wrt_world(box_from_2_, box_size, x_from_2_, y_from_2_);
            get_box_location_wrt_world(box_to_2_, box_size, x_to_2_, y_to_2_);
            get_box_location_wrt_world(box_from_3_, box_size, x_from_3_, y_from_3_);
            get_box_location_wrt_world(box_to_3_, box_size, x_to_3_, y_to_3_);

            y_to_1_ += (player_side_ == "white") ? -(dv + 2) * box_size : (dv + 2) * box_size;

            if (player_side_ == "white")
            {
                x_from_1_ -= robot_position_wrt_world_x_;
                x_to_1_   -= robot_position_wrt_world_x_;
                x_from_2_ -= robot_position_wrt_world_x_;
                x_to_2_   -= robot_position_wrt_world_x_;
                x_from_3_ -= robot_position_wrt_world_x_;
                x_to_3_   -= robot_position_wrt_world_x_;
            }
            else
            {
                x_from_1_ = -x_from_1_ - robot_position_wrt_world_x_;
                y_from_1_ = -y_from_1_;
                x_to_1_   = -x_to_1_  - robot_position_wrt_world_x_;
                y_to_1_   = -y_to_1_;
                x_from_2_ = -x_from_2_ - robot_position_wrt_world_x_;
                y_from_2_ = -y_from_2_;
                x_to_2_   = -x_to_2_  - robot_position_wrt_world_x_;
                y_to_2_   = -y_to_2_;
                x_from_3_ = -x_from_3_ - robot_position_wrt_world_x_;
                y_from_3_ = -y_from_3_;
                x_to_3_   = -x_to_3_  - robot_position_wrt_world_x_;
                y_to_3_   = -y_to_3_;
            }
            y_from_2_ += box_size * 2;
            y_to_3_ += box_size * 2;
            count_promotion_++;

            calculate_joint_angles(x_from_1_, y_from_1_, length_l1_, length_l2_, q0_from_1_, q2_from_1_);
            calculate_joint_angles(x_to_1_, y_to_1_, length_l1_, length_l2_, q0_to_1_, q2_to_1_);
            calculate_joint_angles(x_from_2_, y_from_2_, length_l1_, length_l2_, q0_from_2_, q2_from_2_);
            calculate_joint_angles(x_to_2_, y_to_2_, length_l1_, length_l2_, q0_to_2_, q2_to_2_);
            calculate_joint_angles(x_from_3_, y_from_3_, length_l1_, length_l2_, q0_from_3_, q2_from_3_);
            calculate_joint_angles(x_to_3_, y_to_3_, length_l1_, length_l2_, q0_to_3_, q2_to_3_);

            setOutput("move_from1", std::vector<double>{q0_from_1_, limit_l1_up_, q2_from_1_, q3_from_1_});
            setOutput("move_from_down1", std::vector<double>{q0_from_1_, limit_l1_down_, q2_from_1_, q3_from_1_});
            setOutput("move_to1", std::vector<double>{q0_to_1_, limit_l1_up_, q2_to_1_, q3_to_1_});
            setOutput("move_to_down1", std::vector<double>{q0_to_1_, limit_l1_down_, q2_to_1_, q3_to_1_});
            setOutput("move_from2", std::vector<double>{q0_from_2_, limit_l1_up_, q2_from_2_, q3_from_2_});
            setOutput("move_from_down2", std::vector<double>{q0_from_2_, limit_l1_down_, q2_from_2_, q3_from_2_});
            setOutput("move_to_down2", std::vector<double>{q0_to_2_, limit_l1_down_, q2_to_2_, q3_to_2_});
            setOutput("move_to2", std::vector<double>{q0_to_2_, limit_l1_up_, q2_to_2_, q3_to_2_});
            setOutput("move_from3", std::vector<double>{q0_from_3_, limit_l1_up_, q2_from_3_, q3_from_3_});
            setOutput("move_from_down3", std::vector<double>{q0_from_3_, limit_l1_down_, q2_from_3_, q3_from_3_});
            setOutput("move_to_down3", std::vector<double>{q0_to_3_, limit_l1_down_, q2_to_3_, q3_to_3_});
            setOutput("move_to3", std::vector<double>{q0_to_3_, limit_l1_up_, q2_to_3_, q3_to_3_});
        }

        fen_desired_ = apply_move_to_fen(fen_validated_, move_best_);

        // Remember, but do not write, where this move should leave the board. The
        // next tick commits it to the history file only if the camera agrees --
        // see the comment on fen_pending_ for why writing it here would be wrong.
        fen_pending_ = fen_desired_;

        setOutput("hand_close_position", std::vector<double>{gap_eef_close_, gap_eef_close_, gap_eef_close_});
        setOutput("hand_open_position", std::vector<double>{gap_eef_open_, gap_eef_open_, gap_eef_open_});
        setOutput("move_type", move_type_);

        if (move_type_ == "straight")
        {
            setOutput("move_count", 1);
        }
        else if (move_type_ == "promotion_capture")
        {
            setOutput("move_count", 3);
        }
        else
        {
            setOutput("move_count", 2);
        }

        ///////////////////////////////////////////////////////////

        return BT::NodeStatus::SUCCESS;
    }

    bool MakeDecisionNode::confirm_pending_against_camera()
    {
        if (fen_pending_.empty())
            return false;

        const std::string pending_board = fen_pending_.substr(0, fen_pending_.find(' '));

        // The board is exactly where our move should have left it: the move was
        // executed and the opponent has not replied yet.
        if (pending_board == fen_from_camera_)
        {
            write_to_text_file(fen_pending_);
            fen_from_text_ = fen_pending_;
            fen_from_text_only_board_ = pending_board;
            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                        "Own move confirmed by camera. History advanced to: %s", fen_pending_.c_str());
            fen_pending_.clear();
            return true;
        }

        // Our move was executed and the opponent has already replied. Both plies
        // belong in the history, oldest first.
        for (const auto &reply_fen : possible_next_moves_from_valid_fen_(fen_pending_))
        {
            if (reply_fen.substr(0, reply_fen.find(' ')) == fen_from_camera_)
            {
                write_to_text_file(fen_pending_);
                write_to_text_file(reply_fen);
                fen_from_text_ = reply_fen;
                fen_from_text_only_board_ = fen_from_camera_;
                RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                            "Own move confirmed by camera and the opponent has replied. History advanced to: %s",
                            reply_fen.c_str());
                fen_pending_.clear();
                return true;
            }
        }

        // The board does not show what we planned: the move was never executed,
        // it was executed wrongly, or perception is off. The camera is the
        // authority, so drop the expectation and let the ladder below reconcile
        // from the last position the file is sure about.
        RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                    "Pending move is not on the board (pending: %s, camera: %s). Discarding it and reconciling from the history file.",
                    fen_pending_.c_str(), fen_from_camera_.c_str());
        fen_pending_.clear();
        return false;
    }

    std::string MakeDecisionNode::who_is_owner_of_move()
    {
        std::string owner = "";

        fen_from_text_ = get_last_move_from_text();

        if (!getInput("fen", fen_from_camera_) || fen_from_camera_.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger("MakeDecisionNode"),
                         "Failed to get fen input or fen is empty");
            return owner;
        }
        fen_from_text_only_board_ = fen_from_text_.substr(0, fen_from_text_.find(' '));

        // Commit our own last move to the history first, if the camera backs it
        // up. Without this the file never advances on its own and every single
        // tick has to rediscover the position through the search ladder below.
        confirm_pending_against_camera();

        RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                    "Camera FEN: %s", fen_from_camera_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                    "Text FEN:   %s", fen_from_text_.c_str());

        if (fen_from_camera_ == fen_from_text_only_board_)
        {
            owner = side_to_move(fen_from_text_);
            if (owner == player_side_)
            {
                RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                            "Move owner detected from last valid move and camera: %s (same as player side). move will be made", owner.c_str());
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                            "Move owner detected from last valid move and camera: %s (opponent). waiting", owner.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                        "FENs do not match! Camera FEN: %s, Text FEN: %s",
                        fen_from_camera_.c_str(), fen_from_text_only_board_.c_str());
            if (compare_fens_without_classes(fen_from_camera_, fen_from_text_only_board_))
            {
                RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                            "FENs match when ignoring piece classes. Possible OCR misclassification.");
                owner = side_to_move(fen_from_text_);
                RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                            "Move owner detected from bitmask: %s", owner.c_str());
            }
            else
            {
                std::vector<std::string> possible_next_fens = possible_next_moves_from_valid_fen_(fen_from_text_);
                std::string possible_next_fen_only_board = "";
                RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                            "Checking possible next moves from text FEN against camera FEN...");
                for (const auto &possible_fen : possible_next_fens)
                {
                    possible_next_fen_only_board = possible_fen.substr(0, possible_fen.find(' '));
                    if (possible_next_fen_only_board == fen_from_camera_)
                    {
                        owner = side_to_move(possible_fen);
                        RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                                    "Move owner detected from possible next moves. Current owner: %s. Fen: %s", owner.c_str(), possible_fen.c_str());
                        if (owner != player_side_)
                        {
                            RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                                        "Last move from robot not saved to text file. After possible fen checking, the valid fen saved in text file.");
                            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                                        "Self move detected. Current owner: %s. Waiting for next tick to make move", owner.c_str());
                        }
                        else
                        {
                            RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                                        "Opponent move detected. Current owner: %s. Move will be made", owner.c_str());
                        }

                        write_to_text_file(possible_fen);
                        break;
                    }
                }

                // Ladder order matters: an exact match at any depth is stronger
                // evidence than a class-insensitive match at depth one. Trying
                // depth-1 fuzzy first let a single misclassified piece outrank a
                // position the engine can reproduce exactly, and it ran a compare
                // against every legal move on the way there.
                std::vector<std::string> possible_next_fens_second;
                std::string possible_next_fen_only_board_second = "";

                if (owner.empty())
                {
                    RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                                "No first degree match. Checking second degree possible next moves...");
                    for (const auto &possible_fen : possible_next_fens)
                    {
                        possible_next_fens_second = possible_next_moves_from_valid_fen_(possible_fen);
                        for (const auto &possible_fen_second : possible_next_fens_second)
                        {
                            if (possible_fen_second.substr(0, possible_fen_second.find(' ')) == fen_from_camera_)
                            {
                                // possible_fen is the intermediate position the board
                                // passed through; possible_fen_second is what the camera
                                // sees now. Both go into the history, but the owner is
                                // read off the position actually on the board -- never
                                // off a parent.
                                write_to_text_file(possible_fen);
                                write_to_text_file(possible_fen_second);

                                owner = side_to_move(possible_fen_second);
                                RCLCPP_INFO(rclcpp::get_logger("MakeDecisionNode"),
                                            "Move owner detected from second degree possible next moves: %s (%s). Fen: %s",
                                            owner.c_str(),
                                            (owner == player_side_) ? "same as player side, move will be made"
                                                                    : "opponent, waiting",
                                            possible_fen_second.c_str());
                                break;
                            }
                        }
                        if (!owner.empty())
                            break;
                    }
                }

                if (owner.empty())
                {
                    RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                                "No exact match at either depth. Falling back to first degree moves ignoring piece classes.");
                    for (const auto &possible_fen : possible_next_fens)
                    {
                        possible_next_fen_only_board = possible_fen.substr(0, possible_fen.find(' '));
                        if (compare_fens_without_classes(fen_from_camera_, possible_next_fen_only_board))
                        {
                            write_to_text_file(possible_fen);

                            owner = side_to_move(possible_fen);
                            RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                                        "Move owner detected from first degree possible next moves ignoring piece classes: %s (%s). Fen: %s",
                                        owner.c_str(),
                                        (owner == player_side_) ? "same as player side, move will be made"
                                                                : "opponent, waiting",
                                        possible_fen.c_str());
                            break;
                        }
                    }
                }

                if (owner.empty())
                {
                    RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                                "Still no match. Falling back to second degree possible next moves ignoring piece classes.");
                    for (const auto &possible_fen : possible_next_fens)
                    {
                        possible_next_fens_second = possible_next_moves_from_valid_fen_(possible_fen);
                        for (const auto &possible_fen_second : possible_next_fens_second)
                        {
                            possible_next_fen_only_board_second = possible_fen_second.substr(0, possible_fen_second.find(' '));
                            if (compare_fens_without_classes(fen_from_camera_, possible_next_fen_only_board_second))
                            {
                                write_to_text_file(possible_fen);
                                write_to_text_file(possible_fen_second);

                                owner = side_to_move(possible_fen_second);
                                RCLCPP_WARN(rclcpp::get_logger("MakeDecisionNode"),
                                            "Move owner detected from second degree possible next moves ignoring piece classes: %s (%s). Fen: %s",
                                            owner.c_str(),
                                            (owner == player_side_) ? "same as player side, move will be made"
                                                                    : "opponent, waiting",
                                            possible_fen_second.c_str());
                                break;
                            }
                        }
                        if (!owner.empty())
                            break;
                    }
                }
            }
        }

        // Every branch above only assigns to owner; the single exit is here.
        // The exact-match branch used to fall off the end of this non-void
        // function -- undefined behaviour that went unnoticed because the
        // history file never advanced, so that branch was almost never taken.
        return owner;
    }

    void MakeDecisionNode::send_command(const std::string &cmd)
    {
        write(to_sf_[1], cmd.c_str(), cmd.size());
    }

    std::string MakeDecisionNode::read_until(const std::string &stop)
    {
        std::string output;
        char c;

        while (read(from_sf_[0], &c, 1) == 1)
        {
            output += c;
            if (output.find(stop) != std::string::npos)
                break;
        }
        return output;
    }

    std::string MakeDecisionNode::get_last_move_from_text()
    {
        std::ifstream file(last_moves_path);
        std::string line, last;
        while (std::getline(file, line))
            if (!line.empty())
                last = line;
        return last;
    }

    std::vector<std::string> MakeDecisionNode::possible_next_moves_from_valid_fen_(const std::string &fen)
    {
        std::vector<std::string> fens_possible;
        send_command("position fen " + fen + "\n");
        send_command("go perft 1\n");

        std::string output = read_until("Nodes searched:");

        std::stringstream ss(output);
        std::string line;

        while (std::getline(ss, line))
        {
            auto colon = line.find(':');
            if (colon != std::string::npos)
            {
                if (line.find("Nodes searched:") != std::string::npos)
                    break;

                std::string move = line.substr(0, colon);
                if (move.size() == 4 || move.size() == 5)
                {
                    std::string fen_possible = apply_move_to_fen(fen, move);
                    if (!fen_possible.empty())
                        fens_possible.push_back(fen_possible);
                    else
                        RCLCPP_ERROR(rclcpp::get_logger("MakeDecisionNode"),
                                     "Failed to get possible FEN for move: %s", move.c_str());
                }
            }
        }

        return fens_possible;
    }

    std::string MakeDecisionNode::apply_move_to_fen(
        const std::string &base_fen, const std::string &move)
    {
        send_command("position fen " + base_fen + " moves " + move + "\n");
        send_command("d\n");
        std::string dump = read_until("Checkers:");

        std::stringstream ss(dump);
        std::string line;

        while (std::getline(ss, line))
        {
            if (line.rfind("Fen: ", 0) == 0)
                return line.substr(5);
        }

        return "";
    }

    void MakeDecisionNode::reset_values()
    {
        move_count_ = 0;
        fen_from_text_.clear();
        fen_from_camera_.clear();
        fen_from_text_only_board_.clear();
        move_owner_detected_.clear();
        move_best_.clear();
        move_type_.clear();
        fen_validated_.clear();
        box_from_1_.clear();
        box_to_1_.clear();
        x_from_1_ = 0.0f;
        y_from_1_ = 0.0f;
        x_to_1_ = 0.0f;
        y_to_1_ = 0.0f;
        q0_from_1_ = 0.0f;
        q2_from_1_ = 0.0f;
        q0_to_1_ = 0.0f;
        q2_to_1_ = 0.0f;
        q3_from_1_ = 0.0f;
        q3_to_1_ = 0.0f;
        q0_from_2_ = 0.0f;
        q2_from_2_ = 0.0f;
        q0_to_2_ = 0.0f;
        q2_to_2_ = 0.0f;
        q3_from_2_ = 0.0f;
        q3_to_2_ = 0.0f;
        q0_from_3_ = 0.0f;
        q2_from_3_ = 0.0f;
        q0_to_3_ = 0.0f;
        q2_to_3_ = 0.0f;
        q3_from_3_ = 0.0f;
        q3_to_3_ = 0.0f;
        count_white_captured_ = 0;
        count_black_captured_ = 0;
        dv = 0;
        md = 0;
        
    }

    bool MakeDecisionNode::write_to_text_file(const std::string &fen)
    {
        {
            std::ifstream in(last_moves_path, std::ios::ate | std::ios::binary);
            if (in.is_open() && in.tellg() > 0)
            {
                in.seekg(-1, std::ios::end);
                char last_char;
                in.get(last_char);
                if (last_char != '\n')
                {
                    std::ofstream fix(last_moves_path, std::ios::app);
                    fix << '\n';
                }
            }
        }

        std::ofstream file(last_moves_path, std::ios::app);
        if (!file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("MakeDecisionNode"),
                         "Failed to open last moves file for writing: %s", last_moves_path.c_str());
            return false;
        }
        file << fen << std::endl;
        return true;
    }

    std::string MakeDecisionNode::get_move_type(
        const std::string &fen,
        const std::string &move)
    {

        std::string boardPart = fen.substr(0, fen.find(' '));

        std::stringstream ss(fen);
        std::string token;
        std::string activeColor, castleRights, enPassant;
        ss >> token;
        ss >> activeColor;
        ss >> castleRights;
        ss >> enPassant;

        std::string from = move.substr(0, 2);
        std::string to = move.substr(2, 2);

        // Kaynak karedeki tasi bul
        int fromFile = from[0] - 'a';
        int fromRank = 8 - (from[1] - '0');
        int fromIndex = fromRank * 8 + fromFile;
        int idx = 0;
        char sourcePiece = '.';
        for (char c : boardPart)
        {
            if (c == '/')
                continue;
            if (isdigit(c))
            {
                if (idx + (c - '0') > fromIndex)
                    break;
                idx += (c - '0');
            }
            else
            {
                if (idx == fromIndex)
                {
                    sourcePiece = c;
                    break;
                }
                idx++;
            }
        }

        if (sourcePiece == 'K' && from == "e1")
        {
            if (to == "g1" && castleRights.find('K') != std::string::npos)
                return "short_castle";
            if (to == "c1" && castleRights.find('Q') != std::string::npos)
                return "long_castle";
        }

        if (sourcePiece == 'k' && from == "e8")
        {
            if (to == "g8" && castleRights.find('k') != std::string::npos)
                return "short_castle";
            if (to == "c8" && castleRights.find('q') != std::string::npos)
                return "long_castle";
        }

        if (move.size() == 5)
        {
            int file = move[2] - 'a';
            int rank = 8 - (move[3] - '0');
            int targetIndex = rank * 8 + file;
            int currentIndex = 0;
            bool targetOccupied = false;

            for (char c : boardPart)
            {
                if (c == '/')
                    continue;
                if (isdigit(c))
                {
                    if (currentIndex + (c - '0') > targetIndex)
                        break;
                    currentIndex += (c - '0');
                }
                else
                {
                    if (currentIndex == targetIndex)
                    {
                        targetOccupied = true;
                        break;
                    }
                    currentIndex++;
                }
            }

            if (targetOccupied)
                return "promotion_capture";
            return "promotion";
        }

        int file = move[2] - 'a';
        int rank = 8 - (move[3] - '0');
        int targetIndex = rank * 8 + file;

        int currentIndex = 0;
        for (char c : boardPart)
        {
            if (c == '/')
                continue;

            if (isdigit(c))
            {
                int empty = c - '0';
                if (currentIndex + empty > targetIndex)
                    break;
                currentIndex += empty;
            }
            else
            {
                if (currentIndex == targetIndex)
                    return "capture";
                currentIndex++;
            }
        }

        std::string targetSquare = move.substr(2, 2);
        if (targetSquare == enPassant)
            return "en_passant";

        return "straight";
    }

    bool MakeDecisionNode::is_checkmate(const std::string &fen)
    {
        send_command("position fen " + fen + "\n");
        send_command("go perft 1\n");
        std::string output = read_until("Nodes searched:");
        // Read the rest of the line to get the actual number
        char c;
        while (read(from_sf_[0], &c, 1) == 1 && c != '\n')
            output += c;
        return output.find("Nodes searched: 0") != std::string::npos;
    }

    std::string MakeDecisionNode::get_best_move(const std::string &fen)
    {
        if (player_side_ != "white")
        {
            send_command("position fen " + fen + "\n");
            send_command("go depth 20\n");
        }
        else
        {
            send_command("setoption name Skill Level value 5\n");
            send_command("setoption name MultiPV value 3\n");
            send_command("go depth 10\n");
        }

        std::string output = read_until("bestmove ");

        char c;
        std::string move_line;
        while (read(from_sf_[0], &c, 1) == 1 && c != '\n')
            move_line += c;

        std::string move = move_line.substr(0, move_line.find(' '));
        if (!move.empty())
            return move;

        return "";
    }
} // namespace bizon_behavior_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        bizon_behavior_clients::MakeDecisionNode>(
        "MakeDecisionClient");
}