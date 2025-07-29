#include "lib.h"
#include "api.h"

PID::PID(float kP, float kI, float kD, float settle_error, float settle_time, float timeout):
    kP(kP),
    kI(kI),
    kD(kD),
    settle_error(settle_error),
    settle_time(settle_time),
    timeout(timeout)
{
}

float PID::compute(float error) {
    // update settling
    time_spent += dt;
    if (get_settled(error)) {
        return 0;
    }

    integral += error; // integral

    // when error reaches 0, set integral to 0
    if (fabs(integral) < settle_error) {
        integral = 0;
    }

    // handle integral wind-up: 1) limit range of integral build up
    if (fabs(error) > integral_threshold) {
        integral = 0;
    }

    // rate of change of error
    float derivative = error - prev_error;
    prev_error = error;

    float output = kP*error + kI*integral + kD*derivative;

    return output;
}

bool PID::get_settled(float error) {
    // if time is greater than timeout
    if (time_spent > timeout || time_settled > settle_time) {
        return true;
    }
    // if error is smaller than settle error AND if time settled is greater than settle time
    if (fabs(error) < settle_error) {
        time_settled += dt;
    }

    return false;
}

/*
ODOMETRY

class Drivetrain


1. get motor encoder values
2. get change in encoder value -> distance of wheel travel -> store in delta_L, delta_r, delta_s
3. updated previous encoder values
4. calculate total change in encoder values, convert to distance of wheel travel
5. calculate new orientation theta = theta + adsf ( radians )
6. calculate change in angle
7. calculate local offset (case delta theta = 0, just add the right and left)
8. calculate local offset ( safafs )
9. calculate average orientation
10. calculate global offset
11. calculate new absolute position


preset
- left tracker dist from center
- right tracker dist from center
- 


- get left travelled
- get right travelled

compute

return x, y

update_position()
get_position() -> [x,y]
*/

Drivetrain::Drivetrain(std::vector<std::int8_t> left_motor_ports, std::vector<std::int8_t> right_motor_ports, int imu_port, int left_tracker_port, int right_tracker_port, int back_tracker_port, float track_width, float left_tracker_offset, float right_tracker_offset, float back_tracker_offset):
    left_motors(left_motor_ports),
    right_motors(right_motor_ports),
    imu(imu_port),
    left_tracker(left_tracker_port),
    right_tracker(right_tracker_port),
    back_tracker(back_tracker_port),
    track_width(track_width),
    left_tracker_offset(left_tracker_offset),
    right_tracker_offset(right_tracker_offset)
{
};

std::vector<float> Drivetrain::get_position() {
    return {drive_x, drive_y};
};

void Drivetrain::update_position() {
    /*
    1. get motor encoder values
    2. get change in encoder value -> distance of wheel travel -> store in delta_L, delta_r, delta_s
    3. updated previous encoder values
    4. calculate total change in encoder values, convert to distance of wheel travel
    5. calculate new orientation theta = theta + adsf ( radians )
    6. calculate change in angle
    7. calculate local offset (case delta theta = 0, just add the right and left)
    8. calculate local offset ( safafs )
    9. calculate average orientation
    10. calculate global offset
    11. calculate new absolute position
    */
    // updates dt

    // get motor encoder values
    left_pos = left_tracker.get_position();
    right_pos = right_tracker.get_position();

    // get change in encoder values in DISTANCE
    delta_l_dist = (left_pos-prev_left_pos)*wheel_diameter*M_PI/3.6;
    delta_r_dist = (right_pos-prev_right_pos)*wheel_diameter*M_PI/3.6;
    delta_s_dist = (back_pos-prev_b_pos)*wheel_diameter*M_PI/3.6;

    //update prev encoder values
    prev_left_pos = left_pos;
    prev_right_pos = right_pos;

    // total change since last reset
    total_l_dist = (left_pos-reset_l_pos)*wheel_diameter*M_PI/3.6;
    total_r_dist = (right_pos-reset_r_pos)*wheel_diameter*M_PI/3.6;
    total_s_dist = (back_pos-reset_s_pos)*wheel_diameter*M_PI/3.6;

    // calculate new absolute orientation
    orientation = prev_orientation + (delta_l_dist-delta_r_dist)/(left_tracker_offset+right_tracker_offset);

    // calculate change in angle
    delta_theta = orientation - prev_orientation;

    //

    /*
    360 f, 48 min
    2 cup, 2 cup water line
    */
    if (delta_theta == 0) {
        x_offset = delta_s_dist;
        y_offset = average({delta_r_dist, delta_l_dist});
    } else {
        x_offset = 2 * sin(delta_theta/2)*(delta_s_dist/delta_theta + delta_s_dist);
        y_offset = 2 * sin(delta_theta/2)*(delta_r_dist/delta_theta + delta_r_dist);
    }

    // average orientation
    avg_orientation = prev_orientation + delta_theta/2;

    // global offsets by rotating around -m, using a rotation matrix
    x_global_offset = x_offset * cos(avg_orientation) - y_offset * sin(avg_orientation);
    y_global_offset = y_offset * sin(avg_orientation) + x_offset * cos(avg_orientation);

    // update positions
    drive_x += x_global_offset;
    drive_y += y_global_offset;
}

void Drivetrain::reset_position() {
    reset_l_pos = left_tracker.get_position();
    reset_r_pos = right_tracker.get_position();
    reset_s_pos = back_tracker.get_position();

    reset_orientation = imu.get_heading()/180*M_PI;

    drive_x = 0;
    drive_y = 0;

    left_tracker.set_position(0);
    right_tracker.set_position(0);
    back_tracker.set_position(0);
}