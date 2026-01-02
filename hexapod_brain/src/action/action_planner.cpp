/*******************************************************************************
 * Copyright (c) 2021 Christian Stein
 ******************************************************************************/

#include "action/action_planner.hpp"

namespace brain {
CActionPlanner::CActionPlanner(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    handlerSystem_ = std::make_shared<CSystem>(node);
    handlerMovement_ = std::make_shared<CMovement>(node);

    handlers_.push_back(handlerSystem_);
    handlers_.push_back(handlerMovement_);
}

void CActionPlanner::request(std::vector<std::shared_ptr<RequestBase>> requests, Prio prio) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CActionPlanner:: new requests with prio " << (int)prio);

    switch (prio) {
        case Prio::Highest:
            requestsHighPrio_.clear();
            requestsNormalPrio_.clear();
            requestBackground_.clear();
            cancelRunningRequest();
            requestsHighestPrio_.push_back(requests);
            break;

        case Prio::High:
            if (activePrio_ == Prio::Highest) break;
            if (activePrio_ == Prio::High) {
                requestsHighPrio_.push_back(requests);
                break;
            }
            cancelRunningRequest();
            requestsHighPrio_.push_back(requests);
            break;

        case Prio::Normal:
            if (activePrio_ == Prio::Highest) break;
            requestsNormalPrio_.push_back(requests);
            break;

        case Prio::Background:
            if (activePrio_ == Prio::Highest) break;
            requestBackground_ = requests;
            break;

        default:
            RCLCPP_ERROR_STREAM(node_->get_logger(), "unknown prio of the new request");
            break;
    }
}

void CActionPlanner::update() {
    bool isDone = true;
    for (auto handler : handlers_) {
        handler->update();
        if (!handler->done()) {
            isDone = false;
        }
    }
    isDone_ = isDone;
    if (!isDone_) {
        return;
    }

    if (!requestsHighestPrio_.empty()) {
        activeRequests_ = requestsHighestPrio_.front();
        requestsHighestPrio_.pop_front();
        activePrio_ = Prio::Highest;
    } else if (!requestsHighPrio_.empty()) {
        activeRequests_ = requestsHighPrio_.front();
        requestsHighPrio_.pop_front();
        activePrio_ = Prio::High;
    } else if (!requestsNormalPrio_.empty()) {
        activeRequests_ = requestsNormalPrio_.front();
        requestsNormalPrio_.pop_front();
        activePrio_ = Prio::Normal;
    } else {
        activeRequests_ = requestBackground_;
        activePrio_ = Prio::Background;
    }
    execute(activeRequests_);
}

void CActionPlanner::execute(std::vector<std::shared_ptr<RequestBase>>& requests_v) {
    for (const auto& request : requests_v) {
        if (auto requestSystem = std::dynamic_pointer_cast<RequestSystem>(request)) {
            handlerSystem_->run(requestSystem);
        } else if (auto requestMove = std::dynamic_pointer_cast<CRequestMove>(request)) {
            handlerMovement_->run(requestMove);
        }
        // Communication requests ignored (module removed)
    }
}

void CActionPlanner::cancelRunningRequest() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "CActionPlanner::cancelRunningRequest");
    for (auto handler : handlers_) {
        handler->cancel();
    }
}

bool CActionPlanner::done() {
    return isDone_;
}

}  // namespace brain
