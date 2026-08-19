#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__CONDITION_NODE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__CONDITION_NODE_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace bizon_behavior_clients
{
    class ConditionNode : public BT::ConditionNode
    {
    public:
        ConditionNode(
            const std::string &name,
            const BT::NodeConfiguration &config);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("param1", "Description of param1"),
                BT::InputPort<std::string>("param2", "Description of param2"),
            };
        }
    private:
        std::string param1;
        std::string param2;
    };
}
#endif // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__CONDITION_NODE_HPP_