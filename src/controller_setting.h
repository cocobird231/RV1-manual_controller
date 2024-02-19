#pragma once
#include <linux/joystick.h>

#include <string>
#include <fstream>
#include "nlohmann/json.hpp"

#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg_json.h"

#include "vehicle_interfaces/msg/motor_value_range.hpp"

// Declaration.
bool js_event_btn_equal(js_event event1, js_event event2);
bool js_event_trig_equal(js_event event1, js_event event2);

struct JoystickController;
bool ReadJoystickControllerFile(std::string filePath, JoystickController& outInfo);
bool CvtJSONToJoystickController(const nlohmann::json& json, JoystickController& outInfo);


// js_event struct defined under <linux/joystick.h>
// struct js_event {
// 	__u32 time;	/* event timestamp in milliseconds */
// 	__s16 value;	/* value */
// 	__u8 type;	/* event type */
// 	__u8 number;	/* axis/button number */
// };

bool js_event_btn_equal(js_event event1, js_event event2)
{
    return event1.value == event2.value && 
            event1.type == event2.type && 
            event1.number == event2.number;
}

bool js_event_trig_equal(js_event event1, js_event event2)
{
    return event1.type == event2.type && 
            event1.number == event2.number;
}

struct JoystickController
{
    std::string device;// Device path.
    js_event driveTrigger;// Trigger for driving.
    vehicle_interfaces::msg::MappingData driveTriggerMap;// Mapping data for driving trigger.
    js_event reverseTrigger;// Trigger for reversing.
    vehicle_interfaces::msg::MappingData reverseTriggerMap;// Mapping data for reversing trigger.
    js_event steeringTrigger;// Trigger for steering.
    vehicle_interfaces::msg::MappingData steeringTriggerMap;// Mapping data for steering trigger.
    js_event parkBtn;// Parking button.
    js_event releaseParkBtn;// Release parking button.
    js_event steeringModeBtn1;// Steering mode button 1.
    js_event steeringModeBtn2;// Steering mode button 2.
    js_event steeringModeBtn3;// Steering mode button 3.
    js_event steeringModeBtn4;// Steering mode button 4.
};

/**
 * JoystickController functions.
 */
bool ReadJoystickControllerFile(std::string filePath, JoystickController& outInfo)
{
    try
    {
        nlohmann::json json;
        json.update(nlohmann::json::parse(std::ifstream(filePath)));
        return CvtJSONToJoystickController(json, outInfo);
    }
    catch(...)
    {
        return false;
    }
}

bool CvtJSONToJoystickController(const nlohmann::json& json, JoystickController& outInfo)
{
    try
    {
        outInfo.device = json["device"];

        outInfo.driveTrigger = { 0, 0, json["driveTrigger"]["type"], json["driveTrigger"]["number"] };
        for (const auto& i : json["driveTriggerMapping"]["input_vec"].items())
            outInfo.driveTriggerMap.input_vec.push_back(i.value());
        for (const auto& i : json["driveTriggerMapping"]["output_vec"].items())
            outInfo.driveTriggerMap.output_vec.push_back(i.value());
        if (outInfo.driveTriggerMap.input_vec.size() != outInfo.driveTriggerMap.output_vec.size())
            throw std::runtime_error("[CvtJSONToJoystickController] driveTriggerMapping input and output vector size mismatch.");
        else if (outInfo.driveTriggerMap.input_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToJoystickController] driveTriggerMapping input vector size less than 2.");
        else if (outInfo.driveTriggerMap.output_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToJoystickController] driveTriggerMapping output vector size less than 2.");

        outInfo.reverseTrigger = { 0, 0, json["reverseTrigger"]["type"], json["reverseTrigger"]["number"] };
        for (const auto& i : json["reverseTriggerMapping"]["input_vec"].items())
            outInfo.reverseTriggerMap.input_vec.push_back(i.value());
        for (const auto& i : json["reverseTriggerMapping"]["output_vec"].items())
            outInfo.reverseTriggerMap.output_vec.push_back(i.value());
        if (outInfo.reverseTriggerMap.input_vec.size() != outInfo.reverseTriggerMap.output_vec.size())
            throw std::runtime_error("[CvtJSONToJoystickController] reverseTriggerMapping input and output vector size mismatch.");
        else if (outInfo.reverseTriggerMap.input_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToJoystickController] reverseTriggerMapping input vector size less than 2.");
        else if (outInfo.reverseTriggerMap.output_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToJoystickController] reverseTriggerMapping output vector size less than 2.");

        outInfo.steeringTrigger = { 0, 0, json["steeringTrigger"]["type"], json["steeringTrigger"]["number"] };
        for (const auto& i : json["steeringTriggerMapping"]["input_vec"].items())
            outInfo.steeringTriggerMap.input_vec.push_back(i.value());
        for (const auto& i : json["steeringTriggerMapping"]["output_vec"].items())
            outInfo.steeringTriggerMap.output_vec.push_back(i.value());
        if (outInfo.steeringTriggerMap.input_vec.size() != outInfo.steeringTriggerMap.output_vec.size())
            throw std::runtime_error("[CvtJSONToJoystickController] steeringTriggerMapping input and output vector size mismatch.");
        else if (outInfo.steeringTriggerMap.input_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToJoystickController] steeringTriggerMapping input vector size less than 2.");
        else if (outInfo.steeringTriggerMap.output_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToJoystickController] steeringTriggerMapping output vector size less than 2.");

        outInfo.parkBtn = { 0, json["parkBtn"]["value"], json["parkBtn"]["type"], json["parkBtn"]["number"] };
        outInfo.releaseParkBtn = { 0, json["releaseParkBtn"]["value"], json["releaseParkBtn"]["type"], json["releaseParkBtn"]["number"] };

        outInfo.steeringModeBtn1 = { 0, json["steeringModeBtn1"]["value"], json["steeringModeBtn1"]["type"], json["steeringModeBtn1"]["number"] };
        outInfo.steeringModeBtn2 = { 0, json["steeringModeBtn2"]["value"], json["steeringModeBtn2"]["type"], json["steeringModeBtn2"]["number"] };
        outInfo.steeringModeBtn3 = { 0, json["steeringModeBtn3"]["value"], json["steeringModeBtn3"]["type"], json["steeringModeBtn3"]["number"] };
        outInfo.steeringModeBtn4 = { 0, json["steeringModeBtn4"]["value"], json["steeringModeBtn4"]["type"], json["steeringModeBtn4"]["number"] };
    }
    catch(const std::exception& e)
    {
        std::cerr << "[CvtJSONToJoystickController] Caught exception: " << e.what() << '\n';
        return false;
    }
    catch(...)
    {
        std::cerr << "[CvtJSONToJoystickController] Caught unknown exception.";
        return false;
    }
    return true;
}