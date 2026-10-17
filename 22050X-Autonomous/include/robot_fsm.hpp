#pragma once
#include <functional>

enum RobotState {
	Init,
	Loading,
	Picking,
	Navigating,
	Scoring,
	Toggling,
};

inline void HandleStates(
    RobotState& robot_state,
    const std::function<void(RobotState&)> init_fn,
    const std::function<void(RobotState&)> loading_fn,
    const std::function<void(RobotState&)> picking_fn,
    const std::function<void(RobotState&)> nav_fn,
    const std::function<void(RobotState&)> score_fn,
    const std::function<void(RobotState&)> toggle_fn
)
{
    switch (robot_state)
    {
        case Init:
            break;
        case Loading:
            break;
        case Picking:
            break;
        case Navigating:
            break;
        case Scoring:
            break;
        case Toggling:
            break;
    }
}