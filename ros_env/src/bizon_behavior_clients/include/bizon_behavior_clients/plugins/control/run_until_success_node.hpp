#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RUN_UNTIL_SUCCESS_NODE_HPP
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RUN_UNTIL_SUCCESS_NODE_HPP


#include "behaviortree_cpp/control_node.h"


namespace bizon_behavior_tree
{
class RunUntilSuccessNode : public BT::ControlNode
{
public:
    RunUntilSuccessNode(
        const std::string& name,
        const BT::NodeConfiguration& conf);
    
    ~RunUntilSuccessNode() override = default;

    static BT::PortsList providedPorts()
    {
        return {
        };   
    }

private:
    unsigned int current_child_idx_;

    BT::NodeStatus tick() override;

    void halt() override;
};
}

#endif // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONTROL__RUN_UNTIL_SUCCESS_NODE_HPP