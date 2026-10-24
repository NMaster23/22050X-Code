#pragma once
#include <iostream>
#include <vector>

struct Point {
    std::string obj_type;
    float x;
    float y;
};

inline std::vector<Point> field;

inline void addPoint(int rawX, int rawY, const std::string& label) {
    field.emplace_back(rawX, rawY, label);
}