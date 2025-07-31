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

    // update local offsets
    if (delta_theta == 0) {
        x_offset = delta_s_dist;
        y_offset = average({delta_r_dist, delta_l_dist});
    } else {
        x_offset = 2 * sin(delta_theta/2)*(delta_s_dist/delta_theta + left_tracker_offset);
        y_offset = 2 * sin(delta_theta/2)*(delta_r_dist/delta_theta + right_tracker_offset);
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

PurePursuit::PurePursuit(std::vector<Point> path, float lookahead_distance, float max_angular_velocity = 0, float desired_linear_velocity = 0):
    waypoints(path),
    lookahead_distance(lookahead_distance)
{
    if (path.size() <= 1) { // path must be at least 2 points
        waypoints.insert(waypoints.begin(), {0,0});
    }
};

std::vector<Point> PurePursuit::get_intersection(Point current_pos, Point pt1, Point pt2, double lookahead_radius) {
    std::vector<Point> line = get_line();

    float x2 = pt2.x - current_pos.x;
    float x1 = pt1.x - current_pos.x;
    float y2 = pt2.y - current_pos.y;
    float y1 = pt1.y - current_pos.y;

    float dx = x2-x1;
    float dy = y2-y1;
    float dr = sqrt(pow(dx, 2) + pow(dy, 2));

    float determinant = x1*y2 - x2*y1;

    float discriminant = pow(lookahead_radius, 2)*pow(dr,2)-pow(discriminant, 2);

    float poi_x1 = (determinant*dy + sgn(dy)*dx*sqrt(discriminant))/pow(dr, 2);
    float poi_x2 = (determinant*dy - sgn(dy)*dx*sqrt(discriminant))/pow(dr, 2);
    float poi_y1 = (-determinant*dx + abs(dy)*sqrt(discriminant))/pow(dr, 2);
    float poi_y2 = (-determinant*dx - abs(dy)*sqrt(discriminant))/pow(dr, 2);

    std::vector<Point> intersection_list = {};

    if (discriminant > 0) {

        // 2 intersection
        if (poi_x1 >= std::min(x1, x2) && poi_x1 <= std::max(x1, x2) && poi_y1 >= std::min(y1, y2) && poi_y1 <= std::max(y1, y2)) {
            // POI 1 is within bounds
            // if POI 2 is out of bounds
            intersection_list.push_back({poi_x1, poi_y1});
        }

        if (poi_x2 >= std::min(x1, x2) && poi_x2 <= std::max(x1, x2) && poi_y2 >= std::min(y1, y2) && poi_y2 <= std::max(y1, y2)) {
            // POI 2 is within bounds
            intersection_list.push_back({poi_x2, poi_y2});
        }

        return {{poi_x1, poi_y1}, {poi_x2, poi_y2}};
    } else if (discriminant == 0) {
        if (poi_x1 >= std::min(x1, x2) && poi_x1 <= std::max(x1, x2) && poi_y1 >= std::min(y1, y2) && poi_y1 <= std::max(y1, y2)) {
            // POI is within bounds, POI 1 = POI 2
            intersection_list.push_back({poi_x1, poi_y1});
        }
    }

    // if discriminant < 0
    // if POI is not within bounds
    return intersection_list;
}

std::vector<Point> PurePursuit::get_line() {
   return {waypoints[waypoint_index], waypoints[waypoint_index-1]};
}

Point PurePursuit::set_goal_point(std::vector<Point> intersections, Point current_pos) {
    int size = intersections.size();

    if (size == 2) {
        if (distance(intersections[0], waypoints[waypoint_index]) < distance(intersections[1], waypoints[waypoint_index])) {
            // closer to waypoint
            if (distance(current_pos, waypoints[waypoint_index]) < distance(intersections[0], waypoints[waypoint_index])) {
                waypoint_index++;
            } else {
                goal_point = intersections[0];
            }
        } else {
            if (distance(current_pos, waypoints[waypoint_index]) < distance(intersections[1], waypoints[waypoint_index])) {
                waypoint_index++;
            } else {
                goal_point = intersections[1];
            }
        }
    } else if (size == 1) {
        goal_point = intersections[0];
    }
    return goal_point;
}

Point PurePursuit::get_goal_point() {
    return goal_point;
}

/**
 * Move to the set goal point
 *
 * This function should be called continuously to move the robot to the set goal point.
 * The function will change the motor velocities to move the robot to the goal point.
 *
 * The function will use the current position and the goal point to calculate the curvature.
 * The function will then set the motor velocities based on the curvature.
 */
std::vector<float> PurePursuit::compute_errors(Pose current_pose) {
    /*
    current heading
    current position
    goal point

    deteremine linear error & turn error
    and plug into pid
    and the robot drives
    */

    float dx = goal_point.x - current_pose.x;
    float dy = goal_point.y - current_pose.y;

    float linear_error = distance(goal_point, {current_pose.x, current_pose.y});
    float turn_error = min_angle(atan(dy/dx));
    return {linear_error, turn_error};
}
double average(std::vector<double> list) {
    double sum = 0;
    for (int i = 0; i < list.size(); i++) {
        sum += list[i];
    }
    return sum/list.size();
}

int sgn(float num) {
    if (num > 0) {
        return -1;
    } else {
        return 1;
    }
}

double distance(Point one, Point two) {
    return sqrt(pow((two.x-one.x)+(two.y-one.y), 2));
}

double min_angle(double angle) {

    // limitations: angles greater than 360 or less than -360
    if (angle > 180) {
        return angle - 360;
    } else if (angle < -180) {
        return angle + 360;
    }
    return angle;
}