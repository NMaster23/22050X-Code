#pragma once
#include<string>
#include <iostream>
#include <vector>

struct Point {
    std::string obj_type;
    float x;
    float y;
};

struct Target {
	int id;
	float center_x;
	float center_y;
	float edge_x;
	float edge_y;
	float width;
	float height;
	int confidence;
};

inline std::vector<Point> field;

inline void addPoint(float rawX, float rawY, const std::string& label) {
    field.emplace_back(label, rawX, rawY);
}

inline std::vector<Target> objects;

inline void addObj(Target obj) {
    objects.push_back(obj);
}