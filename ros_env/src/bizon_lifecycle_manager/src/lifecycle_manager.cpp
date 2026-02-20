#include "bizon_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::msg::State;
using bizon_util::LifecycleServiceClient;


namespace bizon_lifecycle_manager
{
LifecycleManager::LifecycleManager(const rclcpp::NodeOptions& options)
: Node("lifecycle_manager", options), diagnostics_updater_(this)
{
    RCLCPP_INFO(get_logger(), "Initializing Bizon LifecycleManager");
    
    // The list of names is parameterized, allowing this module to be used with a different set of nodes
    declare_parameter("node_names", rclcpp::PARAMETER_STRING_ARRAY);
    declare_parameter("autostart", rclcpp::ParameterValue(false));
    declare_parameter("bond_timeout", 4.0);
    declare_parameter("bond_respawn_max_duration", 10.0);
    declare_parameter("attempt_respawn_reconnection", true);
    registerRclPreshutdownCallback();

    //node_names_ = get_parameter("node_names").as_string_array();
    node_names_ = {"behavior_server"};
    //node_names_ = {"bt_navigator"};
    //node_names_ = {"bt_navigator", "behavior_server"};
    RCLCPP_WARN(
        get_logger(),
        "Static node_names_ ititialized, should be parameterized.");
    RCLCPP_INFO(
        get_logger(),
        "Node count : %zu",
        node_names_.size());

    get_parameter("autostart", autostart_);
    double bond_timeout_s;
    get_parameter("bond_timeout", bond_timeout_s);
    bond_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(bond_timeout_s));

    double respawn_timeout_s;
    get_parameter("bond_respawn_max_duration", respawn_timeout_s);
    bond_respawn_max_duration_ = rclcpp::Duration::from_seconds(respawn_timeout_s);

    get_parameter("attempt_respawn_reconnection", attempt_respawn_reconnection_);

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    manager_srv_ = create_service<ManageLifecycleNodes>(
        get_name() + std::string("/manage_nodes"),
        std::bind(&LifecycleManager::managerCallback, this, _1, _2, _3),
        rclcpp::ServicesQoS().get_rmw_qos_profile(),
        callback_group_);

    is_active_srv_ = create_service<std_srvs::srv::Trigger>(
        get_name() + std::string("/is_active"),
        std::bind(&LifecycleManager::isActiveCallback, this, _1, _2, _3),
        rclcpp::ServicesQoS().get_rmw_qos_profile(),
        callback_group_);

    transition_state_map_[Transition::TRANSITION_CONFIGURE] = State::PRIMARY_STATE_INACTIVE;
    transition_state_map_[Transition::TRANSITION_CLEANUP] = State::PRIMARY_STATE_UNCONFIGURED;
    transition_state_map_[Transition::TRANSITION_ACTIVATE] = State::PRIMARY_STATE_ACTIVE;
    transition_state_map_[Transition::TRANSITION_DEACTIVATE] = State::PRIMARY_STATE_INACTIVE;
    transition_state_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
        State::PRIMARY_STATE_FINALIZED;

    transition_label_map_[Transition::TRANSITION_CONFIGURE] = std::string("Configuring ");
    transition_label_map_[Transition::TRANSITION_CLEANUP] = std::string("Cleaning up ");
    transition_label_map_[Transition::TRANSITION_ACTIVATE] = std::string("Activating ");
    transition_label_map_[Transition::TRANSITION_DEACTIVATE] = std::string("Deactivating ");
    transition_label_map_[Transition::TRANSITION_UNCONFIGURED_SHUTDOWN] =
        std::string("Shutting down ");

    init_timer_ = this->create_wall_timer(
        0s,
        [this]() -> void {
        init_timer_->cancel();
        createLifecycleServiceClients();
        if (autostart_) {
            init_timer_ = this->create_wall_timer(
            0s,
            [this]() -> void {
                init_timer_->cancel();
                startup();
            },
            callback_group_);
        }
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_callback_group(callback_group_, get_node_base_interface());
        service_thread_ = std::make_unique<bizon_util::NodeThread>(executor);
        });
    diagnostics_updater_.setHardwareID("Bizon");
    diagnostics_updater_.add("Bizon Health", this, &LifecycleManager::CreateActiveDiagnostic);
    RCLCPP_INFO(get_logger(), "Created %s", get_name());
}

LifecycleManager::~LifecycleManager()
{
    RCLCPP_INFO(get_logger(), "Destroying %s", get_name());
    service_thread_.reset();
}

void LifecycleManager::managerCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<ManageLifecycleNodes::Request> request,
    std::shared_ptr<ManageLifecycleNodes::Response> response)
{
    switch (request->command) {
        case ManageLifecycleNodes::Request::STARTUP:
            response->success = startup();
            break;
        case ManageLifecycleNodes::Request::RESET:
            response->success = reset();
            break;
        case ManageLifecycleNodes::Request::SHUTDOWN:
            response->success = shutdown();
            break;
        case ManageLifecycleNodes::Request::PAUSE:
            response->success = pause();
            break;
        case ManageLifecycleNodes::Request::RESUME:
            response->success = resume();
            break;
    }
}

void LifecycleManager::isActiveCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = system_active_;
}


void LifecycleManager::CreateActiveDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    if (system_active_) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Bizon is active");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Bizon is inactive");
    }
}


void LifecycleManager::createLifecycleServiceClients()
{
  RCLCPP_INFO(get_logger(), "Creating and initializing lifecycle service clients");
  for (auto & node_name : node_names_) {
    node_map_[node_name] = std::make_shared<LifecycleServiceClient>(node_name, shared_from_this());
    RCLCPP_INFO(get_logger(), "Created lifecycle service client for node: %s", node_name.c_str());
  }
}


void LifecycleManager::destroyLifecycleServiceClients()
{
    RCLCPP_DEBUG(get_logger(), "Destroying lifecycle service clients");
    for (auto & kv : node_map_) {
        kv.second.reset();
    }
}


bool LifecycleManager::createBondConnection(const std::string & node_name)
{
    const double timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(bond_timeout_).count();
    const double timeout_s = timeout_ns / 1e9;

    if (bond_map_.find(node_name) == bond_map_.end() && bond_timeout_.count() > 0.0) {
        bond_map_[node_name] = std::make_shared<bond::Bond>("bond", node_name, shared_from_this());
        bond_map_[node_name]->setHeartbeatTimeout(timeout_s);
        bond_map_[node_name]->setHeartbeatPeriod(0.10);
        bond_map_[node_name]->start();
    
        if (!bond_map_[node_name]->waitUntilFormed(rclcpp::Duration(rclcpp::Duration::from_nanoseconds(timeout_ns / 2))))
        {
            RCLCPP_ERROR(get_logger(), "Server %s was unable to be reached after %.2fs by bond. This server may be misconfigured.", node_name.c_str(), timeout_s);
            return false;
        }
        RCLCPP_INFO(get_logger(), "Server %s connected with bond.", node_name.c_str());
    }
    return true;
}

bool LifecycleManager::changeStateForNode(const std::string & node_name, std::uint8_t transition)
{
    RCLCPP_DEBUG(get_logger(), "Changing state for node: %s to %u", node_name.c_str(), transition);

    if (!node_map_[node_name]->change_state(transition) || !(node_map_[node_name]->get_state() == transition_state_map_[transition]))
    {
        RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s", node_name.c_str());
        return false;
    }

    if (transition == Transition::TRANSITION_ACTIVATE) {
        return createBondConnection(node_name);
    } else if (transition == Transition::TRANSITION_DEACTIVATE) {
        bond_map_.erase(node_name);
    }

    return true;
}

bool LifecycleManager::changeStateForAllNodes(std::uint8_t transition, bool hard_change)
{
    // Hard change will continue even if a node fails
    if (transition == Transition::TRANSITION_CONFIGURE || transition == Transition::TRANSITION_ACTIVATE)
    {
        for (auto & node_name : node_names_) {
            try {
                if (!changeStateForNode(node_name, transition) && !hard_change) {
                    return false;
                }
            } catch (const std::runtime_error & e) {
                RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s. Exception: %s", node_name.c_str(), e.what());
                return false;
            }
        }
    } else {
        std::vector<std::string>::reverse_iterator rit;
        for (rit = node_names_.rbegin(); rit != node_names_.rend(); ++rit) {
            try {
                if (!changeStateForNode(*rit, transition) && !hard_change) {
                    return false;
                }
            } catch (const std::runtime_error & e) {
                RCLCPP_ERROR(get_logger(), "Failed to change state for node: %s. Exception: %s", rit->c_str(), e.what());
                return false;
            }
        }
    }
    return true;
}


void LifecycleManager::shutdownAllNodes()
{
    RCLCPP_INFO(get_logger(), "Deactivating, cleaning up, and shutting down nodes");
    changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE);
    changeStateForAllNodes(Transition::TRANSITION_CLEANUP);
    changeStateForAllNodes(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}


bool LifecycleManager::startup()
{
    RCLCPP_INFO(get_logger(), "Starting managed nodes bringup");
    if (!changeStateForAllNodes(Transition::TRANSITION_CONFIGURE) || !changeStateForAllNodes(Transition::TRANSITION_ACTIVATE))
    {
        RCLCPP_ERROR(get_logger(), "Failed to bring up all requested nodes. Aborting bringup.");
        return false;
    }
    RCLCPP_INFO(get_logger(), "Managed nodes are active");
    system_active_ = true;
    createBondTimer();
    return true;
}

bool LifecycleManager::shutdown()
{
    system_active_ = false;
    destroyBondTimer();

    RCLCPP_INFO(get_logger(), "Shutting down managed nodes");
    shutdownAllNodes();
    destroyLifecycleServiceClients();
    RCLCPP_INFO(get_logger(), "Managed nodes have been shut down");
    return true;
}


bool LifecycleManager::reset(bool hard_reset)
{
    system_active_ = false;
    destroyBondTimer();

    RCLCPP_INFO(get_logger(), "Resetting managed nodes");
    // Should transition in reverse order
    if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE, hard_reset) || !changeStateForAllNodes(Transition::TRANSITION_CLEANUP, hard_reset))
    {
        if (!hard_reset) {
            RCLCPP_ERROR(get_logger(), "Failed to reset nodes: aborting reset");
            return false;
        }
    }

    RCLCPP_INFO(get_logger(), "Managed nodes have been reset");
    return true;
}

bool LifecycleManager::pause()
{
    system_active_ = false;
    destroyBondTimer();

    RCLCPP_INFO(get_logger(), "Pausing managed nodes");
    if (!changeStateForAllNodes(Transition::TRANSITION_DEACTIVATE)) {
        RCLCPP_ERROR(get_logger(), "Failed to pause nodes: aborting pause");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Managed nodes have been paused");
    return true;
}


bool LifecycleManager::resume()
{
    RCLCPP_INFO(get_logger(), "Resuming managed nodes");
    if (!changeStateForAllNodes(Transition::TRANSITION_ACTIVATE)) {
        RCLCPP_ERROR(get_logger(), "Failed to resume nodes: aborting resume");
        return false;
    }

    RCLCPP_INFO(get_logger(), "Managed nodes are active");
    system_active_ = true;
    createBondTimer();
    return true;
}

void LifecycleManager::createBondTimer()
{
    if (bond_timeout_.count() <= 0) {
        return;
    }

    RCLCPP_INFO(get_logger(), "Creating bond timer");
    bond_timer_ = this->create_wall_timer(
        200ms,
        std::bind(&LifecycleManager::checkBondConnections, this),
        callback_group_);
}


void LifecycleManager::destroyBondTimer()
{
    if (bond_timer_) {
        RCLCPP_INFO(get_logger(), "Terminating bond timer");
        bond_timer_->cancel();
        bond_timer_.reset();
    }
}


void LifecycleManager::onRclPreshutdown()
{
    RCLCPP_INFO(get_logger(), "Running Bizon LifecycleManager rcl preshutdown: %s", this->get_name());

    destroyBondTimer();

    /*
     * Dropping the bond map is what we really need here, but we drop the others
     * to prevent the bond map being used. Likewise, squash the service thread.
    */
    service_thread_.reset();
    node_names_.clear();
    node_map_.clear();
    bond_map_.clear();
}


void LifecycleManager::registerRclPreshutdownCallback()
{
    rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

    context->add_pre_shutdown_callback(std::bind(&LifecycleManager::onRclPreshutdown, this));
}


void
LifecycleManager::checkBondConnections()
{
    if (!system_active_ || !rclcpp::ok() || bond_map_.empty()) {
        return;
    }

    for (auto & node_name : node_names_) {
        if (!rclcpp::ok()) {
            return;
        }

        if (bond_map_[node_name]->isBroken()) {
            RCLCPP_ERROR(get_logger(), "Have not received a heartbeat from %s.", node_name.c_str());
            RCLCPP_ERROR(get_logger(), "CRITICAL FAILURE: SERVER %s IS DOWN after not receiving a heartbeat for %ld ms. Shutting down related nodes.", node_name.c_str(), bond_timeout_.count());
            reset(true);  // hard reset to transition all still active down
            // if a server crashed, it won't get cleared due to failed transition, clear manually
            bond_map_.clear();

            // Initialize the bond respawn timer to check if server comes back online
            // after a failure, within a maximum timeout period.
            if (attempt_respawn_reconnection_) {
                bond_respawn_timer_ = this->create_wall_timer(
                1s,
                std::bind(&LifecycleManager::checkBondRespawnConnection, this),
                callback_group_);
            }
            return;
        }
    }
}


void
LifecycleManager::checkBondRespawnConnection()
{
    // First attempt in respawn, start maximum duration to respawn
    if (bond_respawn_start_time_.nanoseconds() == 0) {
        bond_respawn_start_time_ = now();
    }

    // Note: system_active_ is inverted since this should be in a failure
    // condition. If another outside user actives the system again, this should not process.
    if (system_active_ || !rclcpp::ok() || node_names_.empty()) {
        //bond_respawn_start_time_ = rclcpp::Time(0);
        bond_respawn_timer_.reset();
        return;
    }

    // Check number of live connections after a bond failure
    int live_servers = 0;
    const int max_live_servers = node_names_.size();
    for (auto & node_name : node_names_) {
        if (!rclcpp::ok()) {
            return;
        }

        try {
            node_map_[node_name]->get_state();  // Only won't throw if the server exists
            live_servers++;
        } catch (...) {
            break;
        }
    }

    // If all are alive, kill timer and retransition system to active
    // Else, check if maximum timeout has occurred
    if (live_servers == max_live_servers) {
        RCLCPP_INFO(get_logger(), "Successfully re-established connections from server respawns, starting back up.");
        //bond_respawn_start_time_ = rclcpp::Time(0);
        bond_respawn_timer_.reset();
        startup();
    } else if (now() - bond_respawn_start_time_ >= bond_respawn_max_duration_) {
        RCLCPP_ERROR(get_logger(), "Failed to re-establish connection from a server crash after maximum timeout.");
        //bond_respawn_start_time_ = rclcpp::Time(0);
        bond_respawn_timer_.reset();
    }
}

}  // namespace bizon_lifecycle_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bizon_lifecycle_manager::LifecycleManager)