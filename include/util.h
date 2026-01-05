#pragma once
#include "api.h"

double average(std::vector<double> list);
int sgn(float num);
double distance(Point one, Point two);
double min_angle(double angle);
double min_angle(double angle);
Point POI(Point one_line[2], Point two_line[2]);

// Matrix
float det_mat_77(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix);
float trace_mat_77(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix);
extern std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> identity_mat;

std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> add_mat(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> one, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> two);
std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> multiply_const_mat(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix, float scale);