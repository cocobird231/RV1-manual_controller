#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <cmath>

#include <string>
#include <vector>
#include <deque>
#include <array>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/control.h"
#include "vehicle_interfaces/utils.h"

#include "vehicle_interfaces/msg/control_chassis.hpp"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/msg/control_steering_wheel.hpp"

#include "vehicle_interfaces/srv/controller_info_reg.hpp"
#include "vehicle_interfaces/srv/controller_info_req.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_reg.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_req.hpp"

#include "controller_setting.h"

using namespace std::chrono_literals;

class Params : public vehicle_interfaces::GenericParams
{
private:
    // Callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr _paramsCallbackHandler;
    std::function<void(const rclcpp::Parameter)> cbFunc_;
    std::atomic<bool> cbFuncSetF_;

public:
    int msg_type = 0;
    int service_mode = 0;
    std::string service_name = "controller";

    double timeout_ms = 30.0;
    double period_ms = 50.0;
    int privilege = 100;
    int pub_type = 0;

    std::string mode = "joystick";
    std::string joystickConfigFile = "controller_joystick.json";
    std::string keyboardConfigFile = "controller_keyboard.json";

private:
    // Fixme:
    void _getParams()
    {
        this->get_parameter("msg_type", this->msg_type);
        this->get_parameter("service_mode", this->service_mode);
        this->get_parameter("service_name", this->service_name);
        this->get_parameter("timeout_ms", this->timeout_ms);
        this->get_parameter("period_ms", this->period_ms);
        this->get_parameter("privilege", this->privilege);
        this->get_parameter("pub_type", this->pub_type);
        this->get_parameter("mode", this->mode);
        this->get_parameter("joystickConfigFile", this->joystickConfigFile);
        this->get_parameter("keyboardConfigFile", this->keyboardConfigFile);
    }

    rcl_interfaces::msg::SetParametersResult _paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult ret;
        ret.successful = true;
        ret.reason = "";

        if (!this->cbFuncSetF_)
            return ret;

        for (const auto& param : params)
        {
            try
            {
                this->cbFunc_(param);
            }
            catch (...)
            {
                ret.successful = false;
                ret.reason = "[Params::_paramsCallback] Caught Unknown Exception!";
            }
        }

        return ret;
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName), 
        cbFuncSetF_(false)
    {
        this->declare_parameter<int>("msg_type", this->msg_type);
        this->declare_parameter<int>("service_mode", this->service_mode);
        this->declare_parameter<std::string>("service_name", this->service_name);
        this->declare_parameter<double>("timeout_ms", this->timeout_ms);
        this->declare_parameter<double>("period_ms", this->period_ms);
        this->declare_parameter<int>("privilege", this->privilege);
        this->declare_parameter<int>("pub_type", this->pub_type);
        this->declare_parameter<std::string>("mode", this->mode);
        this->declare_parameter<std::string>("joystickConfigFile", this->joystickConfigFile);
        this->declare_parameter<std::string>("keyboardConfigFile", this->keyboardConfigFile);
        this->_getParams();

        this->_paramsCallbackHandler = this->add_on_set_parameters_callback(std::bind(&Params::_paramsCallback, this, std::placeholders::_1));
    }

    void addCallbackFunc(const std::function<void(const rclcpp::Parameter)>& callback)
    {
        this->cbFunc_ = callback;
        this->cbFuncSetF_ = true;
    }
};



/**
 * TODO:
 * Only support joystick mode now.
 * Only support steering wheel control now.
 * Only support pwm mode now.
 * [ ] loadJoystickConfig function.
 * [ ] Joystick signal detect loop function.
 * [ ] Joystick analog value mapping.
 * [ ] clear joystick fp.
 * [ ] loadKeyboardConfig function.
 * [ ] Keyboard signal detect loop function.
 * [ ] Keyboard signal convert.
 * [ ] clear keyboard fp.
 */
class Controller : public vehicle_interfaces::VehicleServiceNode
{
private:
    std::shared_ptr<Params> params_;// Controller parameters.
    std::shared_ptr<vehicle_interfaces::Controller> controller_;// Communicate with ControlServerController.
    rclcpp::executors::SingleThreadedExecutor* executor_;// Executor for Controller.
    std::thread* execTh_;// Executor thread.
    vehicle_interfaces::msg::ControlSteeringWheel controlSteeringWheelMsg_;// ControlSteeringWheel message.
    std::mutex controlSteeringWheelMsgLock_;// Lock controlSteeringWheelMsg_.

    // Joystick control.
    FILE* joystick_;// Joystick file pointer.
    std::atomic<bool> joystickF_;// Joystick file open flag.
    JoystickController jInfo_;// Joystick information.
    std::thread* joystickTh_;// Joystick thread.
    
    // Node control.
    std::atomic<bool> exitF_;

private:
    template <typename T>
    void _safeSave(T* ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr = value;
    }

    template <typename T>
    T _safeCall(const T* ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr;
    }

    /**
     * (Sub-thread) Loop function for joystick signal input.
     * Function stops when exitF_ set to true.
     */
    void _joystickTh()
    {
        while (!this->exitF_)
        {
            // Open joystick.
            if (!this->joystickF_)
            {
                if (this->joystick_ != nullptr)
                    fclose(this->joystick_);
                this->joystick_ = fopen(this->jInfo_.device.c_str(), "rb+");
                if (!this->joystick_)
                {
                    this->joystickF_ = false;
                    RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickCbFunc] Open %s failed.\n", this->jInfo_.device.c_str());
                    std::this_thread::sleep_for(1s);
                    continue;
                }
                this->joystickF_ = true;
            }

            // Read joystick data.
            js_event msg;
            const size_t ret = fread(&msg, sizeof(js_event), 1, this->joystick_);
            if (ret == 1)
            {
                printf("time: %-5d, value: %-5d, type: %-5d, number: %-5d\n", msg.time, msg.value, msg.type, msg.number);
                if (msg.type == 2 && js_event_trig_equal(msg, this->jInfo_.driveTrigger))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_DRIVE;
                    this->controlSteeringWheelMsg_.pedal_throttle = vehicle_interfaces::LinearMapping1d(msg.value, 
                                                                    this->jInfo_.driveTriggerMap.input_vec[0], 
                                                                    this->jInfo_.driveTriggerMap.input_vec[1],
                                                                    this->jInfo_.driveTriggerMap.output_vec[0],
                                                                    this->jInfo_.driveTriggerMap.output_vec[1]);
                }
                else if (msg.type == 2 && js_event_trig_equal(msg, this->jInfo_.reverseTrigger))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_REVERSE;
                    this->controlSteeringWheelMsg_.pedal_throttle = vehicle_interfaces::LinearMapping1d(msg.value, 
                                                                    this->jInfo_.reverseTriggerMap.input_vec[0], 
                                                                    this->jInfo_.reverseTriggerMap.input_vec[1],
                                                                    this->jInfo_.reverseTriggerMap.output_vec[0],
                                                                    this->jInfo_.reverseTriggerMap.output_vec[1]);
                }
                else if (msg.type == 2 && js_event_trig_equal(msg, this->jInfo_.steeringTrigger))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.steering = vehicle_interfaces::LinearMapping1d(msg.value, 
                                                                this->jInfo_.steeringTriggerMap.input_vec[0], 
                                                                this->jInfo_.steeringTriggerMap.input_vec[1],
                                                                this->jInfo_.steeringTriggerMap.output_vec[0],
                                                                this->jInfo_.steeringTriggerMap.output_vec[1]);
                }
                else if (msg.type == 1 && js_event_btn_equal(msg, this->jInfo_.parkBtn))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_PARK;
                }
                else if (msg.type == 1 && js_event_btn_equal(msg, this->jInfo_.releaseParkBtn))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_NEUTRAL;
                }
                else if (msg.type == 1 && js_event_btn_equal(msg, this->jInfo_.steeringModeBtn1))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.func_0 = 1;
                }
                else if (msg.type == 1 && js_event_btn_equal(msg, this->jInfo_.steeringModeBtn2))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.func_0 = 2;
                }
                else if (msg.type == 1 && js_event_btn_equal(msg, this->jInfo_.steeringModeBtn3))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.func_0 = 3;
                }
                else if (msg.type == 1 && js_event_btn_equal(msg, this->jInfo_.steeringModeBtn4))
                {
                    std::lock_guard<std::mutex> _lock(this->controlSteeringWheelMsgLock_);
                    this->controlSteeringWheelMsg_.func_0 = 4;
                }
                else
                    continue;
                this->controller_->setControlSteeringWheelData(this->controlSteeringWheelMsg_);
            }
            else if (feof(this->joystick_))
            {
                this->joystickF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickCbFunc] EoF.");
            }
            else if (ferror(this->joystick_))
            {
                this->joystickF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickCbFunc] Error reading %s.", this->jInfo_.device.c_str());
            }
        }
    }

public:
    Controller(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params), 
        executor_(nullptr), 
        execTh_(nullptr), 
        joystick_(nullptr), 
        joystickF_(false), 
        exitF_(false)
    {
        if (params->mode == "joystick")
        {
            // Check JoystickInfo.
            RCLCPP_INFO(this->get_logger(), "[Controller] Loading joystick file: %s", params->joystickConfigFile.c_str());
            if (ReadJoystickControllerFile(params->joystickConfigFile, this->jInfo_))
            {
                this->joystickTh_ = new std::thread(&Controller::_joystickTh, this);
                RCLCPP_INFO(this->get_logger(), "[Controller] Joystick initialized.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[Controller] Failed to read joystick file: %s", params->joystickConfigFile.c_str());
                return;
            }
        }
        vehicle_interfaces::msg::ControllerInfo cinfo;
        cinfo.msg_type = params->msg_type;
        cinfo.service_mode = params->service_mode;
        cinfo.service_name = params->service_name;
        cinfo.timeout_ms = params->timeout_ms;
        cinfo.period_ms = params->period_ms;
        cinfo.privilege = params->privilege;
        cinfo.pub_type = params->pub_type;
        this->controller_ = std::make_shared<vehicle_interfaces::Controller>(params, cinfo);
        this->executor_ = new rclcpp::executors::SingleThreadedExecutor();
        this->executor_->add_node(this->controller_);
        this->execTh_ = new std::thread(vehicle_interfaces::SpinExecutor, this->executor_, "controller", 1000.0);
    }

    ~Controller()
    {
        this->close();
    }

    void close()
    {
        if (this->exitF_)// Ignore process if called repeatedly.
            return;
        this->exitF_ = true;// All looping process will be braked if exitF_ set to true.
        // Destroy joystick timer.
        if (this->joystickTh_ != nullptr)
        {
            this->joystickTh_->join();
            delete this->joystickTh_;
        }
        // Delete joystick.
        if (this->joystick_ != nullptr)
        {
            fclose(this->joystick_);
        }
        // Destroy executor.
        if (this->execTh_ != nullptr)
        {
            this->executor_->cancel();
            this->execTh_->join();
            delete this->execTh_;
            delete this->executor_;
        }
    }
};
