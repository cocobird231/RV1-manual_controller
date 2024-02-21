#pragma once

#include <string>
#include <fstream>
#include "nlohmann/json.hpp"

#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg_json.h"

#include "vehicle_interfaces/msg/motor_value_range.hpp"



/**
 * JoystickController.
 */

#include <linux/joystick.h>

/**
 * Compare two js_event structs without time.
 */
bool js_event_btn_equal(js_event event1, js_event event2);

/**
 * Compare two js_event structs without time and value.
 */
bool js_event_trig_equal(js_event event1, js_event event2);

/**
 * JoystickController struct for joystick controller functional mapping.
 */
struct JoystickController;

/**
 * Read joystick controller file and convert to JoystickController struct.
 */
bool ReadJoystickControllerFile(std::string filePath, JoystickController& outInfo);

/**
 * Convert JSON to JoystickController struct.
 */
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



/**
 * KeyboardController.
 */

#include <linux/input.h>

/**
 * Compare two input_event structs without time.
 */
bool input_event_btn_equal(input_event event1, input_event event2);

/**
 * Compare two input_event structs without time and value.
 */
bool input_event_btn_equal_wo_value(input_event event1, input_event event2);

/**
 * KeyboardController struct for keyboard controller functional mapping.
 */
struct KeyboardController;

/**
 * Read keyboard controller file and convert to KeyboardController struct.
 */
bool ReadKeyboardControllerFile(std::string filePath, KeyboardController& outInfo);

/**
 * Convert JSON to KeyboardController struct.
 */
bool CvtJSONToKeyboardController(const nlohmann::json& json, KeyboardController& outInfo);


// input_event struct defined under <linux/input.h>
// struct input_event {
// 	struct timeval time;
// 	unsigned short type;
// 	unsigned short code;
// 	unsigned int value;
// };

bool input_event_btn_equal(input_event event1, input_event event2)
{
    return event1.type == event2.type && 
            event1.code == event2.code &&
            event1.value == event2.value;
}

bool input_event_btn_equal_wo_value(input_event event1, input_event event2)
{
    return event1.type == event2.type && 
            event1.code == event2.code;
}

struct KeyboardController
{
    std::string device;// Device path.
    input_event driveBtn;// Button for driving.
    vehicle_interfaces::msg::MappingData driveBtnMap;// Mapping data for driving button.
    input_event reverseBtn;// Button for reversing.
    vehicle_interfaces::msg::MappingData reverseBtnMap;// Mapping data for reversing button.
    input_event leftBtn;// Button for left steering.
    vehicle_interfaces::msg::MappingData leftBtnMap;// Mapping data for left steering button.
    input_event rightBtn;// Button for right steering.
    vehicle_interfaces::msg::MappingData rightBtnMap;// Mapping data for right steering button.
    input_event parkBtn;// Parking button.
    input_event releaseParkBtn;// Release parking button.
    input_event steeringModeBtn1;// Steering mode button 1.
    input_event steeringModeBtn2;// Steering mode button 2.
    input_event steeringModeBtn3;// Steering mode button 3.
    input_event steeringModeBtn4;// Steering mode button 4.
};


bool ReadKeyboardControllerFile(std::string filePath, KeyboardController& outInfo)
{
    try
    {
        nlohmann::json json;
        json.update(nlohmann::json::parse(std::ifstream(filePath)));
        return CvtJSONToKeyboardController(json, outInfo);
    }
    catch(...)
    {
        return false;
    }
}

bool CvtJSONToKeyboardController(const nlohmann::json& json, KeyboardController& outInfo)
{
    try
    {
        outInfo.device = json["device"];

        outInfo.driveBtn = { {0, 0}, json["driveBtn"]["type"], json["driveBtn"]["code"], json["driveBtn"]["value"] };
        for (const auto& i : json["driveBtnMapping"]["input_vec"].items())
            outInfo.driveBtnMap.input_vec.push_back(i.value());
        for (const auto& i : json["driveBtnMapping"]["output_vec"].items())
            outInfo.driveBtnMap.output_vec.push_back(i.value());
        if (outInfo.driveBtnMap.input_vec.size() != outInfo.driveBtnMap.output_vec.size())
            throw std::runtime_error("[CvtJSONToKeyboardController] driveBtnMapping input and output vector size mismatch.");
        else if (outInfo.driveBtnMap.input_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] driveBtnMapping input vector size less than 2.");
        else if (outInfo.driveBtnMap.output_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] driveBtnMapping output vector size less than 2.");
        
        outInfo.reverseBtn = { {0, 0}, json["reverseBtn"]["type"], json["reverseBtn"]["code"], json["reverseBtn"]["value"] };
        for (const auto& i : json["reverseBtnMapping"]["input_vec"].items())
            outInfo.reverseBtnMap.input_vec.push_back(i.value());
        for (const auto& i : json["reverseBtnMapping"]["output_vec"].items())
            outInfo.reverseBtnMap.output_vec.push_back(i.value());
        if (outInfo.reverseBtnMap.input_vec.size() != outInfo.reverseBtnMap.output_vec.size())
            throw std::runtime_error("[CvtJSONToKeyboardController] reverseBtnMapping input and output vector size mismatch.");
        else if (outInfo.reverseBtnMap.input_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] reverseBtnMapping input vector size less than 2.");
        else if (outInfo.reverseBtnMap.output_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] reverseBtnMapping output vector size less than 2.");

        outInfo.leftBtn = { {0, 0}, json["leftBtn"]["type"], json["leftBtn"]["code"], json["leftBtn"]["value"] };
        for (const auto& i : json["leftBtnMapping"]["input_vec"].items())
            outInfo.leftBtnMap.input_vec.push_back(i.value());
        for (const auto& i : json["leftBtnMapping"]["output_vec"].items())
            outInfo.leftBtnMap.output_vec.push_back(i.value());
        if (outInfo.leftBtnMap.input_vec.size() != outInfo.leftBtnMap.output_vec.size())
            throw std::runtime_error("[CvtJSONToKeyboardController] leftBtnMapping input and output vector size mismatch.");
        else if (outInfo.leftBtnMap.input_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] leftBtnMapping input vector size less than 2.");
        else if (outInfo.leftBtnMap.output_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] leftBtnMapping output vector size less than 2.");
        
        outInfo.rightBtn = { {0, 0}, json["rightBtn"]["type"], json["rightBtn"]["code"], json["rightBtn"]["value"] };
        for (const auto& i : json["rightBtnMapping"]["input_vec"].items())
            outInfo.rightBtnMap.input_vec.push_back(i.value());
        for (const auto& i : json["rightBtnMapping"]["output_vec"].items())
            outInfo.rightBtnMap.output_vec.push_back(i.value());
        if (outInfo.rightBtnMap.input_vec.size() != outInfo.rightBtnMap.output_vec.size())
            throw std::runtime_error("[CvtJSONToKeyboardController] rightBtnMapping input and output vector size mismatch.");
        else if (outInfo.rightBtnMap.input_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] rightBtnMapping input vector size less than 2.");
        else if (outInfo.rightBtnMap.output_vec.size() < 2)
            throw std::runtime_error("[CvtJSONToKeyboardController] rightBtnMapping output vector size less than 2.");

        outInfo.parkBtn = { {0, 0}, json["parkBtn"]["type"], json["parkBtn"]["code"], json["parkBtn"]["value"] };
        outInfo.releaseParkBtn = { {0, 0}, json["releaseParkBtn"]["type"], json["releaseParkBtn"]["code"], json["releaseParkBtn"]["value"] };

        outInfo.steeringModeBtn1 = { {0, 0}, json["steeringModeBtn1"]["type"], json["steeringModeBtn1"]["code"], json["steeringModeBtn1"]["value"] };
        outInfo.steeringModeBtn2 = { {0, 0}, json["steeringModeBtn2"]["type"], json["steeringModeBtn2"]["code"], json["steeringModeBtn2"]["value"] };
        outInfo.steeringModeBtn3 = { {0, 0}, json["steeringModeBtn3"]["type"], json["steeringModeBtn3"]["code"], json["steeringModeBtn3"]["value"] };
        outInfo.steeringModeBtn4 = { {0, 0}, json["steeringModeBtn4"]["type"], json["steeringModeBtn4"]["code"], json["steeringModeBtn4"]["value"] };
    }
    catch(const std::exception& e)
    {
        std::cerr << "[CvtJSONToKeyboardController] Caught exception: " << e.what() << '\n';
        return false;
    }
    catch(...)
    {
        std::cerr << "[CvtJSONToKeyboardController] Caught unknown exception.";
        return false;
    }
    return true;
}