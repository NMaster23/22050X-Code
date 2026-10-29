#pragma once
#include <functional>

enum RobotState {
	Init,
	Loading_Stationary,
    Loading_Nav,
	Picking,
	Navigating,
	Scoring,
	Toggling,
    Intake_Dec,
};

inline void HandleStates(
    RobotState& robot_state,
    const std::function<void(RobotState&)> init_fn,
    const std::function<void(RobotState&)> loading_fn,
    const std::function<void(RobotState&)> loading_stat_fn,
    const std::function<void(RobotState&)> picking_fn,
    const std::function<void(RobotState&)> nav_fn,
    const std::function<void(RobotState&)> score_fn,
    const std::function<void(RobotState&)> toggle_fn,
    const std::function<void(RobotState&)> intake_decide_fn
)
{
    switch (robot_state)
    {
        case Init:
            init_fn(robot_state);
            break;
        case Loading_Stationary:
            loading_stat_fn(robot_state);
            break;
        case Loading_Nav:
            loading_fn(robot_state);
            break;
        case Picking:
            picking_fn(robot_state);
            break;
        case Navigating:
            nav_fn(robot_state);
            break;
        case Scoring:
            score_fn(robot_state);
            break;
        case Toggling:
            toggle_fn(robot_state);
            break;
        case Intake_Dec:
            intake_decide_fn(robot_state);
            break;
    }
}