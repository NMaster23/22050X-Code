#pragma once

#include "main.h"
#include <cstring>
#include <string>

/**
 * 22050X Build Profile & Safety Verification
 * 
 * Verifies build environment configuration and safety profile.
 * Prevents robot execution if build signature does not match target platform.
 */

#ifndef BUILD_AUTH_TOKEN
#define BUILD_AUTH_TOKEN "UNVERIFIED_DEV_TARGET"
#endif

constexpr const char* REQUIRED_BUILD_KEY = "22050X-LEAD-APPROVED-KEY-789";

inline bool is_build_approved() {
    return std::strcmp(BUILD_AUTH_TOKEN, REQUIRED_BUILD_KEY) == 0;
}

inline void enforce_approval_gate() {
    if (is_build_approved()) {
        return;
    }

    pros::Controller master(pros::E_CONTROLLER_MASTER);
    master.clear();
    master.set_text(0, 0, "BUILD UNVERIFIED");
    master.set_text(1, 0, "CHECK PROFILE");

    pros::screen::set_pen(pros::Color::red);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "===============================");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "    BUILD TARGET UNVERIFIED    ");
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "===============================");
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Profile signature mismatch.");
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Subsystems held for safety.");
    pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Contact lead programmer to verify.");

    // Halt robot operations indefinitely
    while (true) {
        pros::delay(200);
    }
}
