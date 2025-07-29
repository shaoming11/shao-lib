#pragma once
#include "api.h"

class PID {
    public:
        float kP, kI, kD, settle_error, settle_time, timeout, prev_error, integral, integral_threshold = 0;
        float time_settled, time_spent = 0;
        float dt = 10; // milliseconds
        PID(float kP, float kI, float kD, float settle_error, float settle_time, float timeout);

        float compute(float error);
        bool get_settled(float error);
};

class PurePursuit {
    public:
        PurePursuit();
};

/*
- motors
- imu
- rotation tracking wheels
- left dist from center, right dist from center, back dist from center?
- track width
*/
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

double average(std::vector<double> list) {
    double sum = 0;
    for (int i = 0; i < list.size(); i++) {
        sum += list[i];
    }
    return sum/list.size();
}