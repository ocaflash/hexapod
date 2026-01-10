/*******************************************************************************
 * Copyright (c) 2024 Christian Stein
 ******************************************************************************/

#include "requester/gaitcontroller.hpp"

#include <algorithm>

CGaitController::CGaitController(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<CKinematics> kinematics)
    : node_(node), kinematics_(kinematics) {
    FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME =
        node->declare_parameter<double>("FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME", 30.0);
    GAIT_STEP_LENGTH = node->declare_parameter<double>("GAIT_STEP_LENGTH", 0.025);
    LEG_LIFT_HEIGHT = node->declare_parameter<double>("LEG_LIFT_HEIGHT", 0.03);
    max_linear_vel_ = node->declare_parameter<double>("GAIT_MAX_LINEAR_VEL", 0.8);
    max_angular_vel_ = node->declare_parameter<double>("GAIT_MAX_ANGULAR_VEL", 2.0);
    start_ramp_time_s_ = node->declare_parameter<double>("GAIT_START_RAMP_TIME_S", 0.25);
}

void CGaitController::reset() {
    phase_ = 0.0;
    velocity_ = geometry_msgs::msg::Twist();
    ramp_ = 0.0;
}

void CGaitController::setPhaseNeutral() {
    phase_ = 0.0;
}

void CGaitController::liftLegsTripodGroup(bool isFirstTripod) {
    auto positionLiftedLeg = kinematics_->getLegsPositions();

    const auto groupLiftedLegs = isFirstTripod ? groupFirstTripod_ : groupSecondTripod_;

    for (auto& [legIndex, position] : positionLiftedLeg) {
        if (std::find(groupLiftedLegs.begin(), groupLiftedLegs.end(), legIndex) != groupLiftedLegs.end()) {
            const auto baseFootPos = kinematics_->getLegsStandingPositions().at(legIndex);
            auto liftHight = baseFootPos.z + LEG_LIFT_HEIGHT;
            positionLiftedLeg[legIndex] = CPosition(position.x, position.y, liftHight);
        }
    }
    kinematics_->moveBody(positionLiftedLeg);
}

void CGaitController::updateCombinedTripodGait(const geometry_msgs::msg::Twist& velocity, double dt_s, CPose body) {
    // Protect against weird dt (e.g. long stall) to avoid sudden jumps ("convulsions").
    dt_s = std::clamp(dt_s, 0.0, 0.2);

    // Smooth start after STOP: ramp gait amplitude from 0->1 over a short time window.
    if (start_ramp_time_s_ <= 1e-3) {
        ramp_ = 1.0;
    } else {
        ramp_ = std::min(1.0, ramp_ + (dt_s / start_ramp_time_s_));
    }

    // low-pass filtering the velocity
    const double alpha = 0.2;  // Adjust alpha for filtering strength (0.0 to 1.0)
    velocity_ = lowPassFilterTwist(velocity_, velocity, alpha);
    // RCLCPP_INFO_STREAM(node_->get_logger(), "CGaitController::requestMove: filtered velocity: "
    //                                             << velocity_.linear.x << ", " << velocity_.linear.y << ", "
    //                                             << velocity_.angular.z);

    double linear_x = velocity_.linear.x;
    double linear_y = velocity_.linear.y;
    double angular_z = velocity_.angular.z;

    constexpr double ROTATION_WEIGHT = 0.7;

    // Scale step amplitude by stick deflection (independent of cadence).
    const double lin_mag = std::hypot(linear_x, linear_y);
    const double lin_norm = (max_linear_vel_ > 1e-6) ? std::clamp(lin_mag / max_linear_vel_, 0.0, 1.0) : 0.0;
    const double rot_norm =
        (max_angular_vel_ > 1e-6) ? std::clamp(std::abs(angular_z) / max_angular_vel_, 0.0, 1.0) : 0.0;
    const double input_scale = std::max(lin_norm, rot_norm) * ramp_;

    double combined_mag =
        std::sqrt((linear_x * linear_x) + (linear_y * linear_y) + (ROTATION_WEIGHT * angular_z * angular_z));

    // Avoid division by zero
    if (combined_mag < 1e-6) return;

    double norm_x = linear_x / combined_mag;
    double norm_y = linear_y / combined_mag;
    double norm_rot = angular_z / combined_mag;

    // Advance phase based on velocity AND dt.
    // Previously this was missing dt and caused huge phase jumps at low update rates (e.g. 10Hz),
    // resulting in abrupt leg direction changes and servo "twitching".
    double deltaPhase = FACTOR_VELOCITY_TO_GAIT_CYCLE_TIME * combined_mag * dt_s;
    phase_ += deltaPhase;
    // Keep phase bounded to avoid numeric drift over long runs
    phase_ = std::fmod(phase_, 2.0 * M_PI);

    std::map<ELegIndex, CPosition> targetPositions;

    for (auto& [index, leg] : kinematics_->getLegs()) {
        bool isFirstTripod =
            std::find(groupFirstTripod_.begin(), groupFirstTripod_.end(), index) != groupFirstTripod_.end();

        double phaseOffset = isFirstTripod ? 0.0 : M_PI;
        double phaseWithOffset = phase_ + phaseOffset + M_PI_2;

        double step = (GAIT_STEP_LENGTH * input_scale) * cos(phaseWithOffset);
        double lift = (LEG_LIFT_HEIGHT * input_scale) * std::max(0.0, sin(phaseWithOffset));

        const auto baseFootPos = kinematics_->getLegsStandingPositions().at(index);

        // linear
        double deltaX = norm_x * step;
        double deltaY = norm_y * step;

        // rotation
        double legVecX = baseFootPos.x;
        double legVecY = baseFootPos.y;
        double len = std::sqrt(legVecX * legVecX + legVecY * legVecY);

        double rotX = 0.0;
        double rotY = 0.0;

        if (len > 1e-6) {
            double dirX = -legVecY / len;
            double dirY = legVecX / len;

            rotX = dirX * step * norm_rot;
            rotY = dirY * step * norm_rot;
        }

        CPosition target;
        target.x = baseFootPos.x + deltaX + rotX;
        target.y = baseFootPos.y + deltaY + rotY;
        target.z = baseFootPos.z + lift;

        targetPositions[index] = target;
    }

    kinematics_->moveBody(targetPositions, body);

    constexpr double MAX_HEAD_YAW_AMPLITUDE_DEG = 15.0;
    kinematics_->getHead().degYaw = MAX_HEAD_YAW_AMPLITUDE_DEG * std::sin(phase_);
}
