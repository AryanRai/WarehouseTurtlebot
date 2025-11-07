// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: warehouse_manager_node.cpp
// Author(s): Aryan Rai, Inez Dumas
//
// Description: Implementation of polymorphic warehouse manager node
//              that dynamically manages inspection and delivery robots
//              using polymorphism and dynamic camera system control.

#include "Camera/AprilTagDetector.hpp"
#include "Camera/Camera_Node.hpp"
#include "Camera/ColourDetector.hpp"
#include "Robot/DeliveryRobot.hpp"
#include "Robot/InspectionRobot.hpp"
#include "Robot/WarehouseRobot.hpp"
#include <atomic>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

/**
 * @brief Polymorphic Warehouse Management Node
 *
 * Uses polymorphism with base class pointers to manage different robot types:
 * - WarehouseRobot* mActiveRobot (base class pointer)
 * - Runtime casting to InspectionRobot* or DeliveryRobot*
 * - Polymorphic method calls through virtual functions
 * - Dynamic camera system management
 * - Equivalent functionality to run_autonomous_slam.sh
 */
class PolymorphicWarehouseManager : public rclcpp::Node
{
    public:
        /**
         * @brief Constructs the polymorphic warehouse manager
         */
        PolymorphicWarehouseManager()
            : Node("polymorphic_warehouse_manager"), mActiveRobot(nullptr),
              mCurrentMode(RobotMode::NONE), mCameraActive(false)
        {
            // Declare mode parameter
            this->declare_parameter<std::string>("robot_mode", "none");

            // Create services for dynamic control
            mStartOperationsService =
                this->create_service<std_srvs::srv::Trigger>(
                    "warehouse/start_operations",
                    std::bind(
                        &PolymorphicWarehouseManager::startOperationsCallback,
                        this, std::placeholders::_1, std::placeholders::_2));

            mStopOperationsService =
                this->create_service<std_srvs::srv::Trigger>(
                    "warehouse/stop_operations",
                    std::bind(
                        &PolymorphicWarehouseManager::stopOperationsCallback,
                        this, std::placeholders::_1, std::placeholders::_2));

            mGetStatusService = this->create_service<std_srvs::srv::Trigger>(
                "warehouse/get_status",
                std::bind(&PolymorphicWarehouseManager::getStatusCallback, this,
                          std::placeholders::_1, std::placeholders::_2));

            // Create parameter change callback
            mParameterCallback = this->add_on_set_parameters_callback(
                std::bind(&PolymorphicWarehouseManager::onParameterChange, this,
                          std::placeholders::_1));

            // Create executor for dynamic node management
            mExecutor =
                std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
            mExecutor->add_node(this->get_node_base_interface());

            // Create timer for checking parameter changes
            mTimer = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&PolymorphicWarehouseManager::checkModeParameter,
                          this));

            RCLCPP_INFO(this->get_logger(),
                        " Polymorphic Warehouse Manager initialized");
            RCLCPP_INFO(this->get_logger(),
                        "Available modes: inspection, delivery, manual");
            RCLCPP_INFO(this->get_logger(),
                        "Set mode with: ros2 param set "
                        "/polymorphic_warehouse_manager robot_mode inspection");
            RCLCPP_INFO(this->get_logger(), "Services:");
            RCLCPP_INFO(this->get_logger(), "  - /warehouse/start_operations");
            RCLCPP_INFO(this->get_logger(), "  - /warehouse/stop_operations");
            RCLCPP_INFO(this->get_logger(), "  - /warehouse/get_status");
        }

        /**
         * @brief Main execution loop
         */
        void run() { mExecutor->spin(); }

    private:
        enum class RobotMode
        {
            NONE,
            INSPECTION,
            DELIVERY,
            MANUAL
        };

        // POLYMORPHIC ARCHITECTURE - Base class pointer
        std::shared_ptr<WarehouseRobot>
            mActiveRobot; ///< Base class pointer for polymorphism

        // Current state
        std::atomic<RobotMode> mCurrentMode;
        std::atomic<bool> mCameraActive;

        // ROS2 infrastructure
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> mExecutor;
        rclcpp::TimerBase::SharedPtr mTimer;

        // Services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
            mStartOperationsService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
            mStopOperationsService;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mGetStatusService;

        // Parameter callback
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
            mParameterCallback;

        // Camera nodes (managed separately since not part of robot hierarchy)
        std::shared_ptr<CameraNode> mCameraNode;
        std::shared_ptr<AprilTagDetector> mAprilTagDetector;
        std::shared_ptr<ColourDetector> mColourDetector;

        /**
         * @brief Parameter change callback for mode switching
         */
        rcl_interfaces::msg::SetParametersResult
        onParameterChange(const std::vector<rclcpp::Parameter> &parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto &param : parameters)
            {
                if (param.get_name() == "robot_mode")
                {
                    std::string mode = param.as_string();
                    RCLCPP_INFO(this->get_logger(),
                                " Mode change requested: %s", mode.c_str());
                    // Mode change will be handled by timer callback
                }
            }

            return result;
        }

        /**
         * @brief Timer callback to check for mode parameter changes
         */
        void checkModeParameter()
        {
            std::string currentMode =
                this->get_parameter("robot_mode").as_string();

            // Convert string to enum and check if changed
            RobotMode newMode = RobotMode::NONE;
            if (currentMode == "inspection")
            {
                newMode = RobotMode::INSPECTION;
            }
            else if (currentMode == "delivery")
            {
                newMode = RobotMode::DELIVERY;
            }
            else if (currentMode == "manual")
            {
                newMode = RobotMode::MANUAL;
            }

            // Only switch if mode actually changed
            if (newMode != mCurrentMode.load())
            {
                switchToMode(newMode);
            }
        }

        /**
         * @brief Switches to the specified robot mode using polymorphism
         */
        void switchToMode(RobotMode mode)
        {
            RCLCPP_INFO(this->get_logger(), " Polymorphic mode switching...");

            // Stop current operations and cleanup
            stopAllNodes();

            // Start new mode with polymorphic robot creation
            switch (mode)
            {
            case RobotMode::INSPECTION:
                if (startInspectionMode())
                {
                    mCurrentMode = RobotMode::INSPECTION;
                    RCLCPP_INFO(this->get_logger(),
                                "Inspection mode active (polymorphic)");
                    logPolymorphicInfo();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 " Failed to start inspection mode");
                }
                break;

            case RobotMode::DELIVERY:
                if (startDeliveryMode())
                {
                    mCurrentMode = RobotMode::DELIVERY;
                    RCLCPP_INFO(this->get_logger(),
                                " Delivery mode active (polymorphic)");
                    logPolymorphicInfo();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 " Failed to start delivery mode");
                }
                break;

            case RobotMode::MANUAL:
                mCurrentMode = RobotMode::MANUAL;
                RCLCPP_INFO(this->get_logger(), " Manual mode active");
                break;

            default:
                mCurrentMode = RobotMode::NONE;
                RCLCPP_INFO(this->get_logger(), "️ No active mode");
                break;
            }
        }

        /**
         * @brief Logs polymorphic architecture information
         */
        void logPolymorphicInfo()
        {
            if (mActiveRobot)
            {
                RCLCPP_INFO(this->get_logger(), " POLYMORPHIC ARCHITECTURE:");
                RCLCPP_INFO(this->get_logger(),
                            "   Base pointer: WarehouseRobot*");
                RCLCPP_INFO(this->get_logger(), "   Actual type: %s",
                            mActiveRobot->getType() == RobotType::INSPECTION
                                ? "InspectionRobot"
                                : "DeliveryRobot");
                RCLCPP_INFO(this->get_logger(), "   Virtual calls enabled ");
            }
        }

        /**
         * @brief Service callback to start operations using polymorphism
         */
        void startOperationsCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            (void)request; // Unused

            if (!mActiveRobot)
            {
                response->success = false;
                response->message = "No active robot (base pointer is null)";
                RCLCPP_WARN(this->get_logger(),
                            "️ No active robot for polymorphic call");
                return;
            }

            // POLYMORPHIC CALL - resolves to derived class implementation
            RCLCPP_INFO(
                this->get_logger(),
                " Making polymorphic call: mActiveRobot->startOperations()");

            try
            {
                mActiveRobot->startOperations(); // Virtual function call!

                response->success = true;
                response->message =
                    std::string("Started operations for ") +
                    (mActiveRobot->getType() == RobotType::INSPECTION
                         ? "InspectionRobot"
                         : "DeliveryRobot");

                RCLCPP_INFO(this->get_logger(),
                            " Polymorphic operation started: %s",
                            response->message.c_str());
            }
            catch (const std::exception &e)
            {
                response->success = false;
                response->message =
                    "Failed to start operations: " + std::string(e.what());
                RCLCPP_ERROR(this->get_logger(),
                             " Polymorphic call failed: %s", e.what());
            }
        }

        /**
         * @brief Service callback to stop operations using polymorphism
         */
        void stopOperationsCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            (void)request; // Unused

            if (!mActiveRobot)
            {
                response->success = false;
                response->message = "No active robot (base pointer is null)";
                return;
            }

            // POLYMORPHIC CALL - resolves to derived class implementation
            RCLCPP_INFO(
                this->get_logger(),
                " Making polymorphic call: mActiveRobot->stopOperations()");

            try
            {
                mActiveRobot->stopOperations(); // Virtual function call!

                response->success = true;
                response->message =
                    std::string("Stopped operations for ") +
                    (mActiveRobot->getType() == RobotType::INSPECTION
                         ? "InspectionRobot"
                         : "DeliveryRobot");

                RCLCPP_INFO(this->get_logger(),
                            "️ Polymorphic operation stopped: %s",
                            response->message.c_str());
            }
            catch (const std::exception &e)
            {
                response->success = false;
                response->message =
                    "Failed to stop operations: " + std::string(e.what());
                RCLCPP_ERROR(this->get_logger(),
                             " Polymorphic call failed: %s", e.what());
            }
        }

        /**
         * @brief Service callback to get robot status using polymorphism
         */
        void getStatusCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            (void)request; // Unused

            if (!mActiveRobot)
            {
                response->success = true;
                response->message = "No active robot (mode: none)";
                return;
            }

            // POLYMORPHIC CALLS for status information
            try
            {
                std::string robotType =
                    (mActiveRobot->getType() == RobotType::INSPECTION)
                        ? "InspectionRobot"
                        : "DeliveryRobot";
                bool isOperating = mActiveRobot->isOperating(); // Virtual call
                bool hasValidMap = mActiveRobot->hasValidMap(); // Virtual call

                response->success = true;
                response->message =
                    "Robot: " + robotType +
                    " | Operating: " + (isOperating ? "Yes" : "No") +
                    " | Valid Map: " + (hasValidMap ? "Yes" : "No") +
                    " | Camera: " + (mCameraActive ? "Active" : "Inactive");

                RCLCPP_INFO(this->get_logger(), " Polymorphic status: %s",
                            response->message.c_str());
            }
            catch (const std::exception &e)
            {
                response->success = false;
                response->message =
                    "Failed to get status: " + std::string(e.what());
            }
        }

        /**
         * @brief Starts camera system nodes
         */
        bool startCameraSystem()
        {
            if (mCameraActive)
            {
                return true; // Already running
            }

            try
            {
                // Create camera aggregation node
                mCameraNode = std::make_shared<CameraNode>();
                mExecutor->add_node(mCameraNode);

                // Create AprilTag detector
                mAprilTagDetector = std::make_shared<AprilTagDetector>();
                mExecutor->add_node(mAprilTagDetector);

                // Create color detector (for inspection mode)
                mColourDetector = std::make_shared<ColourDetector>();
                mExecutor->add_node(mColourDetector);

                mCameraActive = true;
                RCLCPP_INFO(this->get_logger(), " Camera system started");
                return true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to start camera system: %s", e.what());
                return false;
            }
        }

        /**
         * @brief Starts inspection mode with polymorphic robot creation
         */
        bool startInspectionMode()
        {
            if (!startCameraSystem())
                return false;

            try
            {
                RCLCPP_INFO(this->get_logger(),
                            " Creating InspectionRobot...");
                auto insp_node =
                    std::make_shared<rclcpp::Node>("inspection_robot_node");

                // Create derived, upcast to base
                mActiveRobot = std::make_shared<InspectionRobot>(insp_node);

                // Add its underlying Node to the executor
                mExecutor->add_node(mActiveRobot->getNode());

                RCLCPP_INFO(this->get_logger(),
                            " Polymorphic InspectionRobot created");
                RCLCPP_INFO(this->get_logger(), "   Base pointer: %p",
                            reinterpret_cast<void *>(mActiveRobot.get()));
                return true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to start inspection mode: %s", e.what());
                return false;
            }
        }

        /**
         * @brief Starts delivery mode with polymorphic robot creation
         */
        bool startDeliveryMode()
        {
            try
            {
                RCLCPP_INFO(this->get_logger(), " Creating DeliveryRobot...");
                auto deliv_node =
                    std::make_shared<rclcpp::Node>("delivery_robot_node");

                // Create derived, upcast to base
                mActiveRobot = std::make_shared<DeliveryRobot>(deliv_node);

                // Add its underlying Node to the executor
                mExecutor->add_node(mActiveRobot->getNode());

                RCLCPP_INFO(this->get_logger(),
                            " Polymorphic DeliveryRobot created");
                RCLCPP_INFO(this->get_logger(), "   Base pointer: %p",
                            reinterpret_cast<void *>(mActiveRobot.get()));
                return true;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to start delivery mode: %s", e.what());
                return false;
            }
        }

        /**
         * @brief Provides type-safe downcasting for inspection-specific
         * operations
         */
        InspectionRobot *getInspectionRobot()
        {
            if (!mActiveRobot ||
                mActiveRobot->getType() != RobotType::INSPECTION)
            {
                return nullptr;
            }

            // SAFE DOWNCAST - WarehouseRobot* → InspectionRobot*
            return static_cast<InspectionRobot *>(mActiveRobot.get());
        }

        /**
         * @brief Provides type-safe downcasting for delivery-specific
         * operations
         */
        DeliveryRobot *getDeliveryRobot()
        {
            if (!mActiveRobot || mActiveRobot->getType() != RobotType::DELIVERY)
            {
                return nullptr;
            }

            // SAFE DOWNCAST - WarehouseRobot* → DeliveryRobot*
            return static_cast<DeliveryRobot *>(mActiveRobot.get());
        }

        /**
         * @brief Stops all active nodes and cleans up polymorphic resources
         */
        void stopAllNodes()
        {
            RCLCPP_INFO(this->get_logger(), " Stopping all active nodes...");

            // Remove and destroy polymorphic robot (base pointer cleanup)
            if (mActiveRobot)
            {
                RCLCPP_INFO(this->get_logger(),
                            " Cleaning up polymorphic robot...");
                RCLCPP_INFO(this->get_logger(), "   Type: %s",
                            mActiveRobot->getType() == RobotType::INSPECTION
                                ? "InspectionRobot"
                                : "DeliveryRobot");

                mExecutor->remove_node(mActiveRobot->getNode());
                mActiveRobot
                    .reset(); // Destroys derived object through base pointer

                RCLCPP_INFO(this->get_logger(),
                            " Polymorphic robot destroyed");
            }

            // Remove and destroy camera nodes
            if (mColourDetector)
            {
                mExecutor->remove_node(mColourDetector);
                mColourDetector.reset();
            }

            if (mAprilTagDetector)
            {
                mExecutor->remove_node(mAprilTagDetector);
                mAprilTagDetector.reset();
            }

            if (mCameraNode)
            {
                mExecutor->remove_node(mCameraNode);
                mCameraNode.reset();
            }

            mCameraActive = false;
            mCurrentMode = RobotMode::NONE;

            RCLCPP_INFO(this->get_logger(),
                        " All nodes stopped and cleaned up");
        }
};

/**
 * @brief Main entry point for dynamic warehouse manager
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto manager = std::make_shared<PolymorphicWarehouseManager>();

        RCLCPP_INFO(rclcpp::get_logger("main"),
                    " Polymorphic Warehouse Manager starting...");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    " Features polymorphic robot management:");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "   - Base class pointer: WarehouseRobot*");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "   - Derived classes: InspectionRobot, DeliveryRobot");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "   - Virtual function calls for unified interface");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "Use ROS parameters and services to control:");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "  ros2 param set /polymorphic_warehouse_manager "
                    "robot_mode inspection");
        RCLCPP_INFO(rclcpp::get_logger("main"),
                    "  ros2 service call /warehouse/start_operations "
                    "std_srvs/srv/Trigger");

        manager->run();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Failed to start warehouse manager: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}