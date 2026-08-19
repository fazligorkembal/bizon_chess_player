#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__MAKE_DECISION_CLIENT_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__MAKE_DECISION_CLIENT_NODE_HPP_

#include <string>
#include <vector>
#include "behaviortree_cpp/action_node.h"

namespace bizon_behavior_clients
{
    class MakeDecisionNode : public BT::SyncActionNode
    {
    public:
        MakeDecisionNode(
            const std::string &name,
            const BT::NodeConfiguration &config);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("player_side", "white or black"),
                BT::InputPort<std::string>("fen", "Current board FEN string"),
                // BT::InputPort<std::string>("fen_desired", "Desired board FEN string after move"), // todo: make this optional and use it for validation
                BT::OutputPort<std::string>("move_type", "Type of the move: straight, capture, en_passant, short_castle, long_castle"),
                BT::OutputPort<int>("move_count" , "Number of moves made"),
                BT::OutputPort<std::vector<double>>("move_from1", "The move's source joint angles"),
                BT::OutputPort<std::vector<double>>("move_from_down1", "The move's source with downward offset joint angles"),
                BT::OutputPort<std::vector<double>>("move_to1", "The move's destination joint angles"),
                BT::OutputPort<std::vector<double>>("move_to_down1", "The move's destination with downward offset joint angles"),
                BT::OutputPort<std::vector<double>>("hand_open_position", "Hand open position"),
                BT::OutputPort<std::vector<double>>("hand_close_position", "Hand close position"),
                BT::OutputPort<std::vector<double>>("move_from2", "The move's source joint angles for second piece in case of castling move"),
                BT::OutputPort<std::vector<double>>("move_from_down2", "The move's source with downward offset joint angles for second piece in case of castling move"),
                BT::OutputPort<std::vector<double>>("move_to2", "The move's destination joint angles for second piece in case of castling move"),
                BT::OutputPort<std::vector<double>>("move_to_down2", "The move's destination with downward offset joint angles for second piece in case of castling move"),
                BT::OutputPort<std::vector<double>>("move_from3", "The move's source joint angles for third piece in case of promotion capture"),
                BT::OutputPort<std::vector<double>>("move_from_down3", "The move's source with downward offset joint angles for third piece in case of promotion capture"),
                BT::OutputPort<std::vector<double>>("move_to3", "The move's destination joint angles for third piece in case of promotion capture"),
                BT::OutputPort<std::vector<double>>("move_to_down3", "The move's destination with downward offset joint angles for third piece in case of promotion capture"),
            };
        }
    private:
        std::string last_moves_path = "/home/user/Documents/bizon_chess_player";
        int to_sf_[2];   // Pipe to send commands to Stockfish
        int from_sf_[2]; // Pipe to read responses from Stockfish
        pid_t pid_;
      
        ////////////////////////////////////////////////////////////////////////
        void send_command(const std::string &cmd);
        std::string read_until(const std::string &stop);
        std::string get_last_move_from_text();
        std::vector<std::string> possible_next_moves_from_valid_fen_(const std::string &fen);
        std::string apply_move_to_fen(const std::string &base_fen, const std::string &move);
        std::string who_is_owner_of_move();
        std::string get_move_type(const std::string &fen, const std::string &move);
        std::string get_best_move(const std::string &fen);
        bool write_to_text_file(const std::string &fen);
        bool is_checkmate(const std::string &fen);
        void reset_values();
        ////////////////////////////////////////////////////////////////////////
        std::string player_side_;
        std::string opponent_side_;
        std::string fen_from_text_;
        std::string fen_from_camera_;
        std::string fen_from_text_only_board_;
        std::string fen_validated_;
        std::string move_owner_detected_;
        std::string move_best_;
        std::string move_type_;
        std::string is_king_killed_ = "";

        int count_white_captured_ = 0;
        int count_black_captured_ = 0;
        int dv = 0;
        int md = 0;
        int move_count_ = 0;

        std::string box_from_1_;
        std::string box_to_1_;
        float x_from_1_, y_from_1_;
        float x_to_1_, y_to_1_;
        float q0_from_1_, q2_from_1_;
        float q0_to_1_, q2_to_1_;
        float q3_from_1_, q3_to_1_; // for future use if needed

        std::string box_from_2_;
        std::string box_to_2_;
        float x_from_2_, y_from_2_;
        float x_to_2_, y_to_2_;
        float q0_from_2_, q2_from_2_;
        float q0_to_2_, q2_to_2_;
        float q3_from_2_, q3_to_2_; // for future use if needed

        std::string box_from_3_;
        std::string box_to_3_;
        float x_from_3_, y_from_3_;
        float x_to_3_, y_to_3_;
        float q0_from_3_, q2_from_3_;
        float q0_to_3_, q2_to_3_;
        float q3_from_3_, q3_to_3_; // for future use if needed

        const float box_size = 0.03718857142f; // todo: make this configurable
        const float robot_position_wrt_world_x_ = -0.3; // todo: make this configurable
        const float length_l1_ = 0.29f; // todo: make this configurable
        const float length_l2_ = 0.18f; // todo: make this configurable
        const float gap_eef_close_ = 0.04f; // todo: make this configurable
        const float gap_eef_open_ = 0.20f; // todo: make this configurable
        const float limit_l1_down_ = 0.155f; // todo: make this configurable
        const float limit_l1_up_ = 0.08f; // todo: make this configurable
        std::string fen_desired_ = "";

        int count_saved_promotion_pieces_ = 2;
        int count_promotion_ = 0;
        ////////////////////////////////////////////////////////////////////////

    };
}

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__MAKE_DECISION_CLIENT_NODE_HPP_
