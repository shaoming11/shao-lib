#pragma once
#include "api.h"

struct Point {
    float x;
    float y;
};

struct Pose {
    float x;
    float y;
    float heading;
};

class PID {
    public:
        float kP, kI, kD, settle_error, settle_time, timeout, prev_error, integral, integral_threshold = 0;
        float time_settled, time_spent = 0;
        float dt = 10; // milliseconds
        PID(float kP, float kI, float kD, float settle_error, float settle_time, float timeout);

        float compute(float error);
        bool get_settled(float error);
};

/*
determine curvature to drive a robot to the goal point

arc that joins current point and goal point is constructed
chord length of this arc is the lookahead distance

1.determine the current location of the vehicle
2.find path point closest to the vehicle
3.find the goal point
4.transofrm the goal point into vehicle coordinates
5.calculate the curvature and request the vehicle to set the steering to that curvature
6.update the vehicle's position

tracker - drives vehicle along old path
planner - finds the path segment through new terrain

*/

// sources: Implementation of the Pure Pursuit Controller, controllerPurePursuit from MATLAB, Basic Pure Puruit from SIGBots Wiki
class PurePursuit {
    private:
        float lookahead_distance = 0;
        float max_angular_velocity = 0;
        float desired_linear_velocity = 0;
        std::vector<Point> waypoints;
        int waypoint_index = 1;
        Point goal_point = {0,0};

    public:
        PurePursuit(std::vector<Point> path, float lookahead_radius, float max_angular_velocity = 0, float desired_linear_velocity = 0);
        Point set_goal_point(std::vector<Point> intersections, Point current_pos);
        Point get_goal_point();
        std::vector<Point> get_intersection(Point current_pos, Point pt1, Point pt2, double lookahead_distance); // line-circle intersection, returns list of intersections
        std::vector<Point> get_line();
        std::vector<float> compute_errors(Pose current_pose);
};

// sources: Introduction to Position Tracking by 5225A the E-bots Pilons
class Drivetrain {
    private:
        pros::MotorGroup left_motors;
        pros::MotorGroup right_motors;
        pros::Rotation left_tracker;
        pros::Rotation right_tracker;
        pros::Rotation back_tracker;
        pros::Imu imu;

        float track_width;
        float left_tracker_offset;
        float right_tracker_offset;
        float back_tracker_offset;
        float back_tracker_offset;
        float wheel_diameter = 2.75;
        float total_l_dist, total_r_dist, total_s_dist, left_pos, right_pos, back_pos, prev_left_pos, prev_right_pos, prev_b_pos, delta_l_dist, delta_r_dist, delta_s_dist, reset_l_pos, reset_r_pos, reset_s_pos = 0;
        float prev_drive_x, prev_drive_y, drive_x, drive_y, x_offset, y_offset, x_global_offset, y_global_offset = 0;
        float avg_orientation, prev_orientation, orientation, reset_orientation, delta_theta = 0;
    public:
        Drivetrain(std::vector<std::int8_t> left_motor_ports, std::vector<std::int8_t> right_motor_ports, int imu_port, int left_tracker_port, int right_tracker_port, int back_tracker_port, float track_width, float left_tracker_offset, float right_tracker_offset, float back_tracker_offset);
        std::vector<float> get_position();
        void update_position();
        void reset_position();

};

// utlis
double average(std::vector<double> list);

int sgn(float num);

double distance(Point one, Point two);

double min_angle(double angle);