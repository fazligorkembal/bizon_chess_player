#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__FOREACH_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__FOREACH_NODE_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"

namespace bizon_behavior_clients
{
    class ForeachNode : public BT::ControlNode
    {
    public:
        ForeachNode(
            const std::string &name,
            const BT::NodeConfiguration &config);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("move_count", "How many times to repeat the loop"),
                BT::InputPort<std::vector<double>>("move_from1", "First move source"),
                BT::InputPort<std::vector<double>>("move_from2", "Second move source"),
                BT::InputPort<std::vector<double>>("move_from3", "Third move source"),
                BT::InputPort<std::vector<double>>("move_from_down1", "First move down destination"),
                BT::InputPort<std::vector<double>>("move_from_down2", "Second move down destination"),
                BT::InputPort<std::vector<double>>("move_from_down3", "Third move down destination"),
                BT::InputPort<std::vector<double>>("move_to1", "First move destination"),
                BT::InputPort<std::vector<double>>("move_to2", "Second move destination"),
                BT::InputPort<std::vector<double>>("move_to3", "Third move destination"),
                BT::InputPort<std::vector<double>>("move_to_down1", "First move down destination"),
                BT::InputPort<std::vector<double>>("move_to_down2", "Second move down destination"),
                BT::InputPort<std::vector<double>>("move_to_down3", "Third move down destination"),

                // Algebraic squares behind each move_fromN/move_toN pair --
                // see decision_action_client_node.hpp. Remapped per
                // iteration exactly like the joint-angle ports above, so
                // MoveSequence can read the current iteration's source and
                // destination square via target_square.
                BT::InputPort<std::string>("box_from1", "First move source square"),
                BT::InputPort<std::string>("box_from2", "Second move source square"),
                BT::InputPort<std::string>("box_from3", "Third move source square"),
                BT::InputPort<std::string>("box_to1", "First move destination square"),
                BT::InputPort<std::string>("box_to2", "Second move destination square"),
                BT::InputPort<std::string>("box_to3", "Third move destination square"),

                BT::OutputPort<std::vector<double>>("move_from", "Current move source"),
                BT::OutputPort<std::vector<double>>("move_from_down", "Current move down destination"),
                BT::OutputPort<std::vector<double>>("move_to", "Current move destination"),
                BT::OutputPort<std::vector<double>>("move_to_down", "Current move down destination"),
                BT::OutputPort<std::string>("box_from", "Current move source square"),
                BT::OutputPort<std::string>("box_to", "Current move destination square"),

            };
        }
    private:
        int count = 0;
        int current_index_ = 0;
    };
}
#endif // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__FOREACH_NODE_HPP_