/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 * Modified: Voltage/temperature monitoring disabled (Maestro doesn't provide this)
 ******************************************************************************/

#include "requester/error_management.hpp"

using namespace hexapod_interfaces::msg;

namespace brain {

CErrorManagement::CErrorManagement(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    // Parameters kept for compatibility but not used with Maestro
    node_->declare_parameter("supply_voltage", 12.0);
    node_->declare_parameter("supply_voltage_low", 10.5);
    node_->declare_parameter("supply_voltage_critical_low", 10.0);
    node_->declare_parameter("servo_voltage", 12.0);
    node_->declare_parameter("servo_voltage_low", 10.5);
    node_->declare_parameter("servo_voltage_critical_low", 10.0);
    node_->declare_parameter("servo_temperature_high", 60.0);
    node_->declare_parameter("servo_temperature_critical_high", 70.0);

    supplyVoltageFiltered_ = 12.0;
    servoVoltageFiltered_ = 12.0;
}

EError CErrorManagement::getErrorServo([[maybe_unused]] const ServoStatus& msg) {
    // Maestro doesn't provide voltage/temperature feedback
    return EError::None;
}

EError CErrorManagement::getStatusServoTemperature([[maybe_unused]] const ServoStatus& msg) {
    return EError::None;
}

EError CErrorManagement::getStatusVoltage([[maybe_unused]] const float voltage,
                                          [[maybe_unused]] const float voltageLow,
                                          [[maybe_unused]] const float voltageCriticalLow) {
    return EError::None;
}

float CErrorManagement::calculateAverageValue(std::deque<float>& filteredValues, float voltage) {
    if (filteredValues.size() > 10) {
        filteredValues.pop_front();
    }
    filteredValues.push_back(voltage);

    float averageValue = 0;
    for (const auto& value : filteredValues) {
        averageValue += value;
    }
    return filteredValues.empty() ? voltage : averageValue / filteredValues.size();
}

EError CErrorManagement::filterSupplyVoltage([[maybe_unused]] float voltage) {
    return EError::None;
}

EError CErrorManagement::filterServoVoltage([[maybe_unused]] const ServoStatus& msg) {
    return EError::None;
}

float CErrorManagement::getFilteredSupplyVoltage() {
    return supplyVoltageFiltered_;
}

float CErrorManagement::getFilteredServoVoltage() {
    return servoVoltageFiltered_;
}

}  // namespace brain
