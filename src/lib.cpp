#include "lib.h"
#include "api.h"
#include "util.h"
#include <cmath>
#include <vector>
#include <algorithm>

// ------------------------LIDAR------------------------------------------------------------------------ 
// Constants for LiDAR localization
const float FIELD_SIZE = 144.0f; // 12 feet in inches
const float FIELD_WIDTH = 144.0f; // 12 feet in inches
const float FIELD_HEIGHT = 144.0f; // 12 feet in inches
const float MAX_LIDAR_RANGE = 78.74f; // ~2000mm in inches
const float ASSOCIATION_THRESHOLD = 11.81f; // ~300mm in inches
const float MIN_ANGLE_COS = 0.17f; // ~80 degrees, avoid grazing angles
const float PI = 3.14159265359f;

// Wall enumeration
enum Wall {
    NORTH,  // y = FIELD_SIZE
    EAST,   // x = FIELD_SIZE
    SOUTH,  // y = 0
    WEST,   // x = 0
    NONE
};

// Helper structure for wall measurements
struct WallMeasurement {
    Wall wall;
    float distance;
    float confidence;
    bool valid;
};

// LiDAR configuration structure
struct LidarConfig {
    float offset_x;    // X offset from robot center (mm)
    float offset_y;    // Y offset from robot center (mm) 
    float angle;       // Mounting angle relative to robot forward (radians)
};

// Helper functions for LiDAR localization
float normalize_angle(float angle) {
    while (angle < 0) angle += 2.0f * PI;
    while (angle >= 2.0f * PI) angle -= 2.0f * PI;
    return angle;
}

// Normalize angle to [-PI, PI] for UKF (better for differences)
float normalize_angle_symmetric(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

float deg_to_rad(float deg) {
    return deg * PI / 180.0f;
}

Wall predict_wall(float lidar_angle) {
    float angle_deg = lidar_angle * 180.0f / PI;
    angle_deg = fmod(angle_deg, 360.0f);
    if (angle_deg < 0) angle_deg += 360.0f;
    
    if (angle_deg >= 315.0f || angle_deg < 45.0f) {
        return NORTH;
    } else if (angle_deg >= 45.0f && angle_deg < 135.0f) {
        return EAST;
    } else if (angle_deg >= 135.0f && angle_deg < 225.0f) {
        return SOUTH;
    } else {
        return WEST;
    }
}

float expected_distance_to_wall(float x, float y, float lidar_angle, Wall wall) {
    float cos_angle = cos(lidar_angle);
    float sin_angle = sin(lidar_angle);
    
    switch(wall) {
        case NORTH:
            if (fabs(cos_angle) < MIN_ANGLE_COS) return -1.0f;
            return (FIELD_SIZE - y) / cos_angle;
        case EAST:
            if (fabs(sin_angle) < MIN_ANGLE_COS) return -1.0f;
            return (FIELD_SIZE - x) / sin_angle;
        case SOUTH:
            if (fabs(cos_angle) < MIN_ANGLE_COS) return -1.0f;
            return y / (-cos_angle);
        case WEST:
            if (fabs(sin_angle) < MIN_ANGLE_COS) return -1.0f;
            return x / (-sin_angle);
        default:
            return -1.0f;
    }
}

WallMeasurement associate_measurement(float distance, float x_est, float y_est, 
                                      float theta_est, LidarConfig lidar_config) {
    WallMeasurement measurement;
    measurement.valid = false;
    measurement.distance = distance;
    measurement.confidence = 0.0f;
    measurement.wall = NONE;
    
    if (distance >= MAX_LIDAR_RANGE || distance <= 0) {
        return measurement;
    }
    
    // Calculate LiDAR global position accounting for offsets
    float cos_robot = cos(theta_est);
    float sin_robot = sin(theta_est);
    float lidar_x = x_est + lidar_config.offset_x * cos_robot - lidar_config.offset_y * sin_robot;
    float lidar_y = y_est + lidar_config.offset_x * sin_robot + lidar_config.offset_y * cos_robot;
    
    float lidar_angle = normalize_angle(theta_est + lidar_config.angle);
    Wall predicted_wall = predict_wall(lidar_angle);
    float expected_dist = expected_distance_to_wall(lidar_x, lidar_y, lidar_angle, predicted_wall);
    
    if (expected_dist < 0) {
        return measurement;
    }
    
    float error = fabs(distance - expected_dist);
    
    if (error < ASSOCIATION_THRESHOLD) {
        measurement.valid = true;
        measurement.wall = predicted_wall;
        
        float cos_angle = cos(lidar_angle);
        float sin_angle = sin(lidar_angle);
        
        float perpendicularity = 0.0f;
        switch(predicted_wall) {
            case NORTH:
            case SOUTH:
                perpendicularity = fabs(cos_angle);
                break;
            case EAST:
            case WEST:
                perpendicularity = fabs(sin_angle);
                break;
            default:
                perpendicularity = 0.0f;
        }
        
        measurement.confidence = perpendicularity * (1.0f - distance / MAX_LIDAR_RANGE);
    }
    
    return measurement;
}

Point extract_position_from_measurement(WallMeasurement meas, float theta_est, 
                                       LidarConfig lidar_config, float curr_x, float curr_y) {
    Point pos = {curr_x, curr_y};
    
    if (!meas.valid) {
        return pos;
    }
    
    float lidar_angle = normalize_angle(theta_est + lidar_config.angle);
    float cos_angle = cos(lidar_angle);
    float sin_angle = sin(lidar_angle);
    float d = meas.distance;
    
    // Calculate LiDAR position from wall measurement
    Point lidar_pos = {curr_x, curr_y};
    
    switch(meas.wall) {
        case NORTH:
            lidar_pos.y = FIELD_SIZE - d * cos_angle;
            lidar_pos.x = curr_x + lidar_config.offset_x * cos(theta_est) - lidar_config.offset_y * sin(theta_est);
            break;
        case EAST:
            lidar_pos.x = FIELD_SIZE - d * sin_angle;
            lidar_pos.y = curr_y + lidar_config.offset_x * sin(theta_est) + lidar_config.offset_y * cos(theta_est);
            break;
        case SOUTH:
            lidar_pos.y = d * (-cos_angle);
            lidar_pos.x = curr_x + lidar_config.offset_x * cos(theta_est) - lidar_config.offset_y * sin(theta_est);
            break;
        case WEST:
            lidar_pos.x = d * (-sin_angle);
            lidar_pos.y = curr_y + lidar_config.offset_x * sin(theta_est) + lidar_config.offset_y * cos(theta_est);
            break;
        default:
            break;
    }
    
    // Convert LiDAR position back to robot center position
    float cos_robot = cos(theta_est);
    float sin_robot = sin(theta_est);
    pos.x = lidar_pos.x - lidar_config.offset_x * cos_robot + lidar_config.offset_y * sin_robot;
    pos.y = lidar_pos.y - lidar_config.offset_x * sin_robot - lidar_config.offset_y * cos_robot;
    
    return pos;
}

// ------------------------LIDAR------------------------------------------------------------------------

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
    this->time_spent += this->dt;
    if (this->get_settled(error)) {
        return 0;
    }

    this->integral += error * this->dt / 1000.0f; // integral (convert ms to seconds)

    // when error reaches 0, set integral to 0
    if (fabs(this->integral) < this->settle_error) {
        this->integral = 0;
    }

    // handle integral wind-up: 1) limit range of integral build up
    if (fabs(error) > this->integral_threshold) {
        this->integral = 0;
    }

    // rate of change of error
    float derivative = (error - this->prev_error) / (this->dt / 1000.0f);
    this->prev_error = error;

    float output = this->kP*error + this->kI*this->integral + this->kD*derivative;

    return output;
}

bool PID::get_settled(float error) {
    if (time_spent >= timeout) return true;

    if (fabs(error) <= settle_error) {
        time_settled += dt;
    } else {
        time_settled = 0;
    }

    return time_settled >= settle_time;
}

void PID::reset() {
    this->time_settled = 0;
    this->time_spent = 0;
    this->integral = 0;
    this->prev_error = 0;
}

Drivetrain::Drivetrain(std::vector<std::int8_t> left_motor_ports, std::vector<std::int8_t> right_motor_ports, PID linear_pid, PID turn_pid, int motor_speed, int imu_port, int left_tracker_port, int right_tracker_port, int back_tracker_port, float track_width, float left_tracker_offset, float right_tracker_offset, float back_tracker_offset):
    left_motors(left_motor_ports),
    right_motors(right_motor_ports),
    linear_pid(linear_pid),
    turn_pid(turn_pid),
    motor_speed(motor_speed),
    imu(imu_port),
    left_tracker(left_tracker_port),
    right_tracker(right_tracker_port),
    back_tracker(back_tracker_port),
    track_width(track_width),
    left_tracker_offset(left_tracker_offset),
    right_tracker_offset(right_tracker_offset)
{
};

Point Drivetrain::get_position() {
    return {drive_x, drive_y};
};

Pose Drivetrain::get_pose() {
    return {drive_x, drive_y, imu.get_heading()};
}

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
    back_pos = back_tracker.get_position();

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

void Drivetrain::move(double left_voltage, double right_voltage) {
    left_motors.move_voltage(left_voltage);
    right_motors.move_voltage(right_voltage);
}

void Drivetrain::move_to_point(Point target) {
    target_point = target;
    path = {get_position(), target_point};

    if (move_task != nullptr) {
        delete(move_task);
    }

    move_task = new pros::Task(move_wrapper, this);
}

void Drivetrain::move_wrapper(void *param) {
    Drivetrain* drive = static_cast<Drivetrain*>(param);
    PurePursuit route(drive->path,  5);
    std::vector<float> errors = route.compute_errors(drive->get_pose());
    while (route.get_settled(drive->get_position()) == false) {
        route.set_goal_point(drive->get_position());
        errors = route.compute_errors(drive->get_pose());
        float linear_voltage = drive->linear_pid.compute(errors[0]);
        float turn_voltage = drive->turn_pid.compute(errors[1]);
        drive->move(linear_voltage+turn_voltage, linear_voltage-turn_voltage);
        pros::delay(10);
    }
}

void Drivetrain::start_odom() {
    if (odom_task != nullptr) {
        delete(odom_task);
    }

    odom_task = new pros::Task(odom_wrapper, this);
}

void Drivetrain::odom_wrapper(void *param) {
    Drivetrain* drive = static_cast<Drivetrain*>(param);
    while (true) {
        drive->update_position();
        pros::delay(10);
    }
} 

PurePursuit::PurePursuit(std::vector<Point> path, float lookahead_distance, float max_angular_velocity, float desired_linear_velocity):
    waypoints(path),
    lookahead_distance(lookahead_distance)
{
    if (path.size() <= 1) { // path must be at least 2 points
        waypoints.insert(waypoints.begin(), {0,0});
    }
};

std::vector<Point> PurePursuit::get_intersection(Point current_pos, Point pt1, Point pt2, double lookahead_radius) {

    float x2 = pt2.x - current_pos.x;
    float x1 = pt1.x - current_pos.x;
    float y2 = pt2.y - current_pos.y;
    float y1 = pt1.y - current_pos.y;

    float dx = x2-x1;
    float dy = y2-y1;
    float dr = sqrt(pow(dx, 2) + pow(dy, 2));

    float determinant = x1*y2 - x2*y1;

    float discriminant = pow(lookahead_radius, 2)*pow(dr,2)-pow(determinant, 2);

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

        return intersection_list;
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

Point PurePursuit::set_goal_point(Point current_pos) {
    std::vector<Point> intersections = get_intersection(current_pos, waypoints[waypoint_index-1], waypoints[waypoint_index], lookahead_distance);
    int size = intersections.size();

    if (size == 2) {
        if (distance(intersections[0], waypoints[waypoint_index]) < distance(intersections[1], waypoints[waypoint_index])) {
            // closer to waypoint
            if (distance(current_pos, waypoints[waypoint_index]) < distance(intersections[0], waypoints[waypoint_index])) {
                if (waypoint_index < waypoints.size() - 1) {
                    waypoint_index++;
                }
            } else {
                goal_point = intersections[0];
            }
        } else {
            if (distance(current_pos, waypoints[waypoint_index]) < distance(intersections[1], waypoints[waypoint_index])) {
                if (waypoint_index < waypoints.size() - 1) {
                    waypoint_index++;
                }
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
    float target_angle = atan2(dy, dx);
    float turn_error = target_angle - current_pose.heading; // heading error
    
    // Normalize angle to [-π, π]
    while (turn_error > M_PI) turn_error -= 2.0 * M_PI;
    while (turn_error < -M_PI) turn_error += 2.0 * M_PI;
    // float curvature = 2*dx/pow(lookahead_distance, 2);
    return {linear_error, turn_error};
}

bool PurePursuit::get_settled(Point current_pos) {
    if (pow((current_pos.x - goal_point.x), 2) + pow((current_pos.y - goal_point.y), 2) <= settle_radius) {
        return true;
    }
    return false;
}

OWDrivetrain::OWDrivetrain(std::vector<std::int8_t> left_motor_ports, std::vector<std::int8_t> right_motor_ports, PID linear_pid, PID turn_pid, int motor_speed, int imu_port, int imu2_port, pros::Rotation* left_tracker_ptr, float track_width, float left_tracker_offset, pros::Distance* left_dist_ptr, pros::Distance* right_dist_ptr, pros::Distance* front_dist_ptr, pros::Distance* back_dist_ptr, UKF* ukf_instance, float front_x_offset, float front_y_offset, float back_x_offset, float back_y_offset, float left_x_offset, float left_y_offset, float right_x_offset, float right_y_offset):
    left_motors(left_motor_ports),
    right_motors(right_motor_ports),
    linear_pid(linear_pid),
    turn_pid(turn_pid),
    motor_speed(motor_speed),
    imu(imu_port),
    imu2(imu2_port),
    left_tracker(left_tracker_ptr),
    track_width(track_width),
    left_tracker_offset(left_tracker_offset),
    sensor_offset_front_x(front_x_offset),
    sensor_offset_front_y(front_y_offset),
    sensor_offset_back_x(back_x_offset),
    sensor_offset_back_y(back_y_offset),
    sensor_offset_left_x(left_x_offset),
    sensor_offset_left_y(left_y_offset),
    sensor_offset_right_x(right_x_offset),
    sensor_offset_right_y(right_y_offset),
    left_dist(left_dist_ptr),
    right_dist(right_dist_ptr),
    front_dist(front_dist_ptr),
    back_dist(back_dist_ptr)
{
    // Store the UKF instance
    ukf = ukf_instance;
};

Point OWDrivetrain::dist_est_pos() {
    // Define LiDAR configurations with physical offsets (you'll need to set these values)
    const float FRONT_OFFSET_X = 0.0f;   // Set actual front LiDAR X offset 
    const float FRONT_OFFSET_Y = 3.94f; // Set actual front LiDAR Y offset (~100mm)
    const float RIGHT_OFFSET_X = 3.94f; // Set actual right LiDAR X offset (~100mm)  
    const float RIGHT_OFFSET_Y = 0.0f;   // Set actual right LiDAR Y offset
    const float BACK_OFFSET_X = 0.0f;    // Set actual back LiDAR X offset
    const float BACK_OFFSET_Y = -3.94f; // Set actual back LiDAR Y offset (~-100mm)
    const float LEFT_OFFSET_X = -3.94f; // Set actual left LiDAR X offset (~-100mm)
    const float LEFT_OFFSET_Y = 0.0f;    // Set actual left LiDAR Y offset
    
    LidarConfig front_config = {FRONT_OFFSET_X, FRONT_OFFSET_Y, 0.0f};           // 0° (forward)
    LidarConfig right_config = {RIGHT_OFFSET_X, RIGHT_OFFSET_Y, -PI / 2.0f};     // -90° (right)
    LidarConfig back_config = {BACK_OFFSET_X, BACK_OFFSET_Y, PI};                // 180° (backward)
    LidarConfig left_config = {LEFT_OFFSET_X, LEFT_OFFSET_Y, PI / 2.0f};         // 90° (left)
    
    // Get current pose estimate (using current position and heading)
    float x_est = drive_x;
    float y_est = drive_y;
    float theta_est = heading * PI / 180.0f; // Convert degrees to radians
    
    // Collect LiDAR samples and convert to inches
    const float MM_TO_INCHES = 1.0f / 25.4f;
    float l = left_dist->get() * MM_TO_INCHES;   // Left LiDAR distance in inches
    float r = right_dist->get() * MM_TO_INCHES;  // Right LiDAR distance in inches
    float f = front_dist->get() * MM_TO_INCHES;  // Front LiDAR distance in inches
    float b = back_dist->get() * MM_TO_INCHES;   // Back LiDAR distance in inches
    
    // Associate each measurement with a wall
    WallMeasurement front_meas = associate_measurement(f, x_est, y_est, theta_est, front_config);
    WallMeasurement right_meas = associate_measurement(r, x_est, y_est, theta_est, right_config);
    WallMeasurement back_meas = associate_measurement(b, x_est, y_est, theta_est, back_config);
    WallMeasurement left_meas = associate_measurement(l, x_est, y_est, theta_est, left_config);
    
    // Extract position estimates from valid measurements
    std::vector<Point> x_measurements;
    std::vector<Point> y_measurements;
    std::vector<float> x_weights;
    std::vector<float> y_weights;
    
    // Process front LiDAR
    if (front_meas.valid) {
        Point pos = extract_position_from_measurement(front_meas, theta_est, front_config, x_est, y_est);
        if (front_meas.wall == NORTH || front_meas.wall == SOUTH) {
            y_measurements.push_back(pos);
            y_weights.push_back(front_meas.confidence);
        } else if (front_meas.wall == EAST || front_meas.wall == WEST) {
            x_measurements.push_back(pos);
            x_weights.push_back(front_meas.confidence);
        }
    }
    
    // Process right LiDAR
    if (right_meas.valid) {
        Point pos = extract_position_from_measurement(right_meas, theta_est, right_config, x_est, y_est);
        if (right_meas.wall == NORTH || right_meas.wall == SOUTH) {
            y_measurements.push_back(pos);
            y_weights.push_back(right_meas.confidence);
        } else if (right_meas.wall == EAST || right_meas.wall == WEST) {
            x_measurements.push_back(pos);
            x_weights.push_back(right_meas.confidence);
        }
    }
    
    // Process back LiDAR
    if (back_meas.valid) {
        Point pos = extract_position_from_measurement(back_meas, theta_est, back_config, x_est, y_est);
        if (back_meas.wall == NORTH || back_meas.wall == SOUTH) {
            y_measurements.push_back(pos);
            y_weights.push_back(back_meas.confidence);
        } else if (back_meas.wall == EAST || back_meas.wall == WEST) {
            x_measurements.push_back(pos);
            x_weights.push_back(back_meas.confidence);
        }
    }
    
    // Process left LiDAR
    if (left_meas.valid) {
        Point pos = extract_position_from_measurement(left_meas, theta_est, left_config, x_est, y_est);
        if (left_meas.wall == NORTH || left_meas.wall == SOUTH) {
            y_measurements.push_back(pos);
            y_weights.push_back(left_meas.confidence);
        } else if (left_meas.wall == EAST || left_meas.wall == WEST) {
            x_measurements.push_back(pos);
            x_weights.push_back(left_meas.confidence);
        }
    }
    
    // Compute weighted average position
    Point result = {x_est, y_est}; // Default to current estimate
    
    // Calculate weighted X position
    if (!x_measurements.empty()) {
        float weighted_x = 0.0f;
        float total_weight = 0.0f;
        
        for (size_t i = 0; i < x_measurements.size(); i++) {
            weighted_x += x_measurements[i].x * x_weights[i];
            total_weight += x_weights[i];
        }
        
        if (total_weight > 0.0f) {
            result.x = weighted_x / total_weight;
        }
    }
    
    // Calculate weighted Y position
    if (!y_measurements.empty()) {
        float weighted_y = 0.0f;
        float total_weight = 0.0f;
        
        for (size_t i = 0; i < y_measurements.size(); i++) {
            weighted_y += y_measurements[i].y * y_weights[i];
            total_weight += y_weights[i];
        }
        
        if (total_weight > 0.0f) {
            result.y = weighted_y / total_weight;
        }
    }
    
    // Boundary check - ensure position is within field
    result.x = std::max(0.0f, std::min(FIELD_SIZE, result.x));
    result.y = std::max(0.0f, std::min(FIELD_SIZE, result.y));
    
    return result;
}

Point OWDrivetrain::tracker_est_pos() {
    left_pos = left_tracker->get_position();
    float distance = left_pos - prev_left_pos;
    prev_left_pos = left_pos;
    float convert_heading = -heading;
    return {static_cast<float>(drive_y + cos(convert_heading)*distance), static_cast<float>(drive_y + sin(convert_heading)*distance)};
}

Point OWDrivetrain::get_position() {
    return {drive_x, drive_y};
}


State OWDrivetrain::get_state() {
    State state;
    state.x = drive_x;
    state.y = drive_y;
    state.heading = heading * M_PI / 180.0f; // Convert to radians
    state.lv = 0; // Linear velocity - would need to be calculated
    state.av = 0; // Angular velocity - would need to be calculated  
    state.imu_bias = 0;
    state.encoder_bias = 0;
    return state;
}

Pose OWDrivetrain::get_pose() {
    return {drive_x, drive_y, heading};
}

void OWDrivetrain::reset_position() {
    drive_x = 0;
    drive_y = 0;
    heading = 0;
    prev_drive_x = 0;
    prev_drive_y = 0;
    left_tracker->set_position(0);
    prev_left_pos = 0;
}

void OWDrivetrain::start_odom() {
    if (odom_task != nullptr) {
        delete(odom_task);
    }
    odom_task = new pros::Task(odom_wrapper, this);
}

void OWDrivetrain::odom_wrapper(void *param) {
    OWDrivetrain* drive = static_cast<OWDrivetrain*>(param);
    while (true) {
        drive->update_position_localization(); // Use the enhanced localization
        
        // Run UKF if it's initialized
        if (drive->ukf != nullptr) {
            // Get actual motor voltages from motor groups
            double left_voltage = drive->left_motors.get_voltage();
            double right_voltage = drive->right_motors.get_voltage();
            double dt = 0.01; // 10ms
            
            // STEP 1: PREDICT (every cycle, fixed dt)
            // Use encoders + IMU (and motion model) to propagate state forward
            drive->ukf->predict(left_voltage, right_voltage, dt);
            
            // STEP 2: BUILD POSE OBSERVATION (only if valid this cycle)
            bool has_valid_observation = drive->update_position_localization_for_ukf();
            
            // STEP 3: CORRECT (immediately after building valid observation)
            if (has_valid_observation) {
                drive->ukf->correct();
            }
            
            // Use UKF estimate to influence position
            State ukf_estimate = drive->ukf->get_estimate();
            drive->drive_x = ukf_estimate.x;
            drive->drive_y = ukf_estimate.y;
            drive->heading = ukf_estimate.heading * 180.0f / M_PI; // Convert back to degrees
        }
        
        pros::delay(10);
    }
}

void OWDrivetrain::move(double left_voltage, double right_voltage) {
    left_motors.move_voltage(left_voltage);
    right_motors.move_voltage(right_voltage);
}

void OWDrivetrain::run_display() {
    
}

void OWDrivetrain::test_lidar_display() {
    // Clear the screen
    pros::lcd::clear();
    
    // Get current pose estimate
    Point current_pos = {drive_x, drive_y};
    
    // Safely get raw sensor readings with error checking
    const float MM_TO_INCHES = 1.0f / 25.4f;
    float l = -1, r = -1, f = -1, b = -1;
    
    try {
        // Get distance sensor readings with bounds checking
        int32_t left_raw = left_dist->get();
        int32_t right_raw = right_dist->get();
        int32_t front_raw = front_dist->get();
        int32_t back_raw = back_dist->get();
        
        // Convert to inches only if readings are valid (positive and reasonable)
        if (left_raw > 0 && left_raw < 4000) l = left_raw * MM_TO_INCHES;
        if (right_raw > 0 && right_raw < 4000) r = right_raw * MM_TO_INCHES;
        if (front_raw > 0 && front_raw < 4000) f = front_raw * MM_TO_INCHES;
        if (back_raw > 0 && back_raw < 4000) b = back_raw * MM_TO_INCHES;
        
    } catch (...) {
        // If any sensor call fails, display error
        pros::lcd::print(0, "=== SENSOR ERROR ===");
        pros::lcd::print(1, "Distance sensor fault");
        return;
    }
    
    // Display current robot state
    pros::lcd::print(0, "=== LiDAR Debug ===");
    pros::lcd::print(1, "Current: X=%.1f Y=%.1f H=%.1f", current_pos.x, current_pos.y, heading);
    pros::lcd::print(4, "Distances: L=%.0f R=%.0f", l, r);
    pros::lcd::print(5, "           F=%.0f B=%.0f", f, b);
}

void OWDrivetrain::move_to_point(Point target) {
    target_point = target;
    path = {get_position(), target_point};

    if (move_task != nullptr) {
        delete(move_task);
    }

    move_task = new pros::Task(move_wrapper, this);
}

void OWDrivetrain::move_wrapper(void *param) {
    OWDrivetrain* drive = static_cast<OWDrivetrain*>(param);
    PurePursuit route(drive->path,  5);
    std::vector<float> errors = route.compute_errors(drive->get_pose());

    drive->linear_pid.reset();
    drive->turn_pid.reset();

    while (route.get_settled(drive->get_position()) == false) {
        route.set_goal_point(drive->get_position());
        errors = route.compute_errors(drive->get_pose());
        float linear_voltage = drive->linear_pid.compute(errors[0]);
        float turn_voltage = drive->turn_pid.compute(errors[1]);
        drive->move(linear_voltage+turn_voltage, linear_voltage-turn_voltage);
        pros::delay(10);
    }
}

void OWDrivetrain::turn_to(double target) {
    turn_target = target;
    turn_error = heading - turn_target;

    if (turn_task != nullptr) {
        delete(turn_task);
    }

    turn_task = new pros::Task(move_wrapper, this);
}

void OWDrivetrain::turn_wrapper(void *param) {
    OWDrivetrain* drive = static_cast<OWDrivetrain*>(param);

    drive->turn_pid.reset();

    while (!drive->turn_pid.get_settled(drive->turn_error)) {
        float turn_voltage = drive->turn_pid.compute(drive->turn_error);
        drive->move(turn_voltage, -turn_voltage);
        drive->turn_error = drive->heading - drive->turn_target;
        pros::delay(10);
    }
}

Measurement OWDrivetrain::get_measurement(State input) {
    /* Get ACTUAL sensor readings from hardware
    Note: The 'input' parameter is ignored - this function reads real sensor values
    For PREDICTED measurements from state estimates, use predict_measurement_from_state()
    */
    
    Measurement measurement;
    
    // Get current sensor readings and convert to inches
    const float MM_TO_INCHES = 1.0f / 25.4f;
    measurement.l = left_dist->get() * MM_TO_INCHES;    // Left distance sensor
    measurement.r = right_dist->get() * MM_TO_INCHES;   // Right distance sensor  
    measurement.f = front_dist->get() * MM_TO_INCHES;   // Front distance sensor
    measurement.b = back_dist->get() * MM_TO_INCHES;    // Back distance sensor
    measurement.rotation = left_tracker->get_position(); // Rotation encoder
    measurement.heading = heading;      // IMU heading
    
    return measurement;
}

std::vector<Measurement> OWDrivetrain::sample_all_sensors(int num_samples, int delay_ms) {
    std::vector<Measurement> samples;
    samples.reserve(num_samples);
    
    for (int i = 0; i < num_samples; i++) {
        Measurement sample;
        
        const float MM_TO_INCHES = 1.0f / 25.4f;
        sample.l = left_dist->get() * MM_TO_INCHES;
        sample.r = right_dist->get() * MM_TO_INCHES;
        sample.f = front_dist->get() * MM_TO_INCHES;
        sample.b = back_dist->get() * MM_TO_INCHES;
        sample.rotation = left_tracker->get_position();
        sample.heading = (min_angle(imu.get_heading()) + min_angle(imu2.get_heading())) / 2.0f;
        
        samples.push_back(sample);
        
        if (i < num_samples - 1) {
            pros::delay(delay_ms);
        }
    }
    
    return samples;
}

std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> OWDrivetrain::calculate_measurement_noise_matrix(const std::vector<Measurement>& samples) {
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> noise_matrix;
    
    if (samples.empty()) {
        for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
            for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
                noise_matrix[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return noise_matrix;
    }
    
    float means[MEASUREMENT_DIMENSIONS] = {0};
    
    for (const auto& sample : samples) {
        means[0] += sample.l;
        means[1] += sample.r;
        means[2] += sample.f;
        means[3] += sample.b;
        means[4] += sample.rotation;
        means[5] += sample.heading;
    }
    
    for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
        means[i] /= samples.size();
    }
    
    for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
        for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
            noise_matrix[i][j] = 0.0f;
        }
    }
    
    for (const auto& sample : samples) {
        float deviations[MEASUREMENT_DIMENSIONS] = {
            sample.l - means[0],
            sample.r - means[1], 
            sample.f - means[2],
            sample.b - means[3],
            sample.rotation - means[4],
            sample.heading - means[5]
        };
        
        for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
            for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
                noise_matrix[i][j] += deviations[i] * deviations[j];
            }
        }
    }
    
    for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
        for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
            noise_matrix[i][j] /= (samples.size() - 1);
        }
    }
    
    return noise_matrix;
}

void OWDrivetrain::update_position_localization() {
    // ========================================
    // STEP 1: IMU Fusion for Heading
    // ========================================
    float imu1_heading = min_angle(imu.get_heading());
    float imu2_heading = min_angle(imu2.get_heading());
    
    bool imu1_valid = !std::isnan(imu1_heading) && std::isfinite(imu1_heading);
    bool imu2_valid = !std::isnan(imu2_heading) && std::isfinite(imu2_heading);
    
    float current_heading = heading; // Default to previous
    if (imu1_valid && imu2_valid) {
        current_heading = (imu1_heading + imu2_heading) / 2.0f;
    } else if (imu1_valid) {
        current_heading = imu1_heading;
    } else if (imu2_valid) {
        current_heading = imu2_heading;
    }
    
    float theta_rad = current_heading * M_PI / 180.0f; // Convert to radians
    
    // ========================================
    // STEP 2: Dead Reckoning with Tracking Wheel
    // ========================================
    float current_tracker_pos = left_tracker->get_position();
    float tracker_delta = current_tracker_pos - prev_left_pos;
    prev_left_pos = current_tracker_pos;
    
    // Convert encoder ticks to distance in inches (wheel_diameter is already in inches)
    float distance_traveled = tracker_delta * wheel_diameter * M_PI / 360.0f;
    
    // Tracking wheel measures perpendicular movement (vertical/strafe)
    // Assuming tracking wheel is perpendicular to robot front
    float dead_reckon_dx = distance_traveled * sin(theta_rad);
    float dead_reckon_dy = distance_traveled * cos(theta_rad);
    
    // Dead reckoning position estimate
    Point dead_reckon_pos = {drive_x + dead_reckon_dx, drive_y + dead_reckon_dy};
    
    // ========================================
    // STEP 3: Distance Sensor Localization
    // ========================================
    
    // Use configured sensor offsets from constructor
    
    // Get distance readings
    float front_reading = front_dist->get();
    float back_reading = back_dist->get();
    float left_reading = left_dist->get();
    float right_reading = right_dist->get();
    
    // Convert sensor readings from mm to inches
    const float MM_TO_INCHES = 1.0f / 25.4f;
    front_reading *= MM_TO_INCHES;
    back_reading *= MM_TO_INCHES;
    left_reading *= MM_TO_INCHES;
    right_reading *= MM_TO_INCHES;
    
    // Validate readings (within sensor range)
    bool front_valid = front_reading > 0.39f && front_reading < MAX_LIDAR_RANGE; // >10mm
    bool back_valid = back_reading > 0.39f && back_reading < MAX_LIDAR_RANGE;
    bool left_valid = left_reading > 0.39f && left_reading < MAX_LIDAR_RANGE;
    bool right_valid = right_reading > 0.39f && right_reading < MAX_LIDAR_RANGE;
    
    // Storage for position estimates from each sensor
    std::vector<float> x_estimates, y_estimates;
    std::vector<float> x_weights, y_weights;
    
    float cos_theta = cos(theta_rad);
    float sin_theta = sin(theta_rad);
    
    // Front sensor (+Y wall constraint)
    if (front_valid) {
        // Rotate sensor offset into field frame
        float offset_field_x = sensor_offset_front_x * cos_theta - sensor_offset_front_y * sin_theta;
        float offset_field_y = sensor_offset_front_x * sin_theta + sensor_offset_front_y * cos_theta;
        
        // Convert wall distance to robot center position
        float estimated_y = FIELD_HEIGHT - front_reading - offset_field_y;
        
        y_estimates.push_back(estimated_y);
        y_weights.push_back(1.0f - front_reading / MAX_LIDAR_RANGE); // Higher weight for closer readings
    }
    
    // Back sensor (-Y wall constraint)  
    if (back_valid) {
        float offset_field_x = sensor_offset_back_x * cos_theta - sensor_offset_back_y * sin_theta;
        float offset_field_y = sensor_offset_back_x * sin_theta + sensor_offset_back_y * cos_theta;
        
        float estimated_y = back_reading - offset_field_y;
        
        y_estimates.push_back(estimated_y);
        y_weights.push_back(1.0f - back_reading / MAX_LIDAR_RANGE);
    }
    
    // Left sensor (-X wall constraint)
    if (left_valid) {
        float offset_field_x = sensor_offset_left_x * cos_theta - sensor_offset_left_y * sin_theta;
        float offset_field_y = sensor_offset_left_x * sin_theta + sensor_offset_left_y * cos_theta;
        
        float estimated_x = left_reading - offset_field_x;
        
        x_estimates.push_back(estimated_x);
        x_weights.push_back(1.0f - left_reading / MAX_LIDAR_RANGE);
    }
    
    // Right sensor (+X wall constraint)
    if (right_valid) {
        float offset_field_x = sensor_offset_right_x * cos_theta - sensor_offset_right_y * sin_theta;
        float offset_field_y = sensor_offset_right_x * sin_theta + sensor_offset_right_y * cos_theta;
        
        float estimated_x = FIELD_WIDTH - right_reading - offset_field_x;
        
        x_estimates.push_back(estimated_x);
        x_weights.push_back(1.0f - right_reading / MAX_LIDAR_RANGE);
    }
    
    // ========================================
    // STEP 4: Weighted Average for Distance-Based Position
    // ========================================
    Point distance_pos = {drive_x, drive_y}; // Default to current position
    
    // Compute weighted average for X position
    if (!x_estimates.empty()) {
        float weighted_sum_x = 0, total_weight_x = 0;
        for (size_t i = 0; i < x_estimates.size(); i++) {
            weighted_sum_x += x_estimates[i] * x_weights[i];
            total_weight_x += x_weights[i];
        }
        if (total_weight_x > 0) {
            distance_pos.x = weighted_sum_x / total_weight_x;
        }
    }
    
    // Compute weighted average for Y position  
    if (!y_estimates.empty()) {
        float weighted_sum_y = 0, total_weight_y = 0;
        for (size_t i = 0; i < y_estimates.size(); i++) {
            weighted_sum_y += y_estimates[i] * y_weights[i];
            total_weight_y += y_weights[i];
        }
        if (total_weight_y > 0) {
            distance_pos.y = weighted_sum_y / total_weight_y;
        }
    }
    
    // ========================================
    // STEP 5: Sensor Fusion (Dead Reckoning + Distance Sensors)
    // ========================================
    bool dead_reckon_valid = !std::isnan(dead_reckon_pos.x) && !std::isnan(dead_reckon_pos.y) &&
                             std::isfinite(dead_reckon_pos.x) && std::isfinite(dead_reckon_pos.y);
    
    bool distance_valid = !x_estimates.empty() || !y_estimates.empty();
    
    if (dead_reckon_valid && distance_valid) {
        // Calculate disagreement between methods
        float position_diff = sqrt(pow(dead_reckon_pos.x - distance_pos.x, 2) + 
                                   pow(dead_reckon_pos.y - distance_pos.y, 2));
        
        // Dynamic weighting based on number of valid sensors and agreement
        int num_valid_sensors = (front_valid ? 1 : 0) + (back_valid ? 1 : 0) + 
                                (left_valid ? 1 : 0) + (right_valid ? 1 : 0);
        
        float distance_weight, dead_reckon_weight;
        
        if (num_valid_sensors >= 3 && position_diff < 3.94f) { // ~100mm in inches
            // High confidence: multiple sensors agree
            distance_weight = 0.8f;
            dead_reckon_weight = 0.2f;
        } else if (num_valid_sensors >= 2 && position_diff < 7.87f) { // ~200mm in inches
            // Medium confidence
            distance_weight = 0.6f;
            dead_reckon_weight = 0.4f;
        } else if (num_valid_sensors >= 1 && position_diff < 15.75f) { // ~400mm in inches
            // Lower confidence
            distance_weight = 0.4f;
            dead_reckon_weight = 0.6f;
        } else {
            // Poor sensor data or large disagreement - trust dead reckoning
            distance_weight = 0.1f;
            dead_reckon_weight = 0.9f;
        }
        
        // Fused position estimate
        drive_x = distance_weight * distance_pos.x + dead_reckon_weight * dead_reckon_pos.x;
        drive_y = distance_weight * distance_pos.y + dead_reckon_weight * dead_reckon_pos.y;
        
    } else if (dead_reckon_valid) {
        // Only dead reckoning available
        drive_x = dead_reckon_pos.x;
        drive_y = dead_reckon_pos.y;
    } else if (distance_valid) {
        // Only distance sensors available
        drive_x = distance_pos.x;
        drive_y = distance_pos.y;
    }
    // Else keep previous position
    
    // Update heading
    heading = current_heading;
    
    // Apply field boundary constraints
    drive_x = std::max(0.0f, std::min(FIELD_WIDTH, drive_x));
    drive_y = std::max(0.0f, std::min(FIELD_HEIGHT, drive_y));
    
    // Store for next iteration
    prev_drive_x = drive_x;
    prev_drive_y = drive_y;
}

bool OWDrivetrain::update_position_localization_for_ukf() {
    // This method computes pose observation z = [x_meas, y_meas, theta_meas] and validates it
    // Returns true if the observation is "valid enough" to trust for UKF correction
    
    // ========================================
    // STEP 1: IMU Fusion for Heading
    // ========================================
    float imu1_heading = min_angle(imu.get_heading());
    float imu2_heading = min_angle(imu2.get_heading());
    
    bool imu1_valid = !std::isnan(imu1_heading) && std::isfinite(imu1_heading);
    bool imu2_valid = !std::isnan(imu2_heading) && std::isfinite(imu2_heading);
    
    // Must have at least one valid IMU for heading observation
    if (!imu1_valid && !imu2_valid) {
        return false; // No valid heading observation
    }
    
    float observed_heading = heading; // Default to previous
    if (imu1_valid && imu2_valid) {
        observed_heading = (imu1_heading + imu2_heading) / 2.0f;
    } else if (imu1_valid) {
        observed_heading = imu1_heading;
    } else if (imu2_valid) {
        observed_heading = imu2_heading;
    }
    
    float theta_rad = observed_heading * M_PI / 180.0f;
    
    // ========================================
    // STEP 2: Distance Sensor Position Observation
    // ========================================
    
    // Get distance readings and validate
    float front_reading = front_dist->get();
    float back_reading = back_dist->get();
    float left_reading = left_dist->get();
    float right_reading = right_dist->get();
    
    // Convert sensor readings from mm to inches
    const float MM_TO_INCHES = 1.0f / 25.4f;
    front_reading *= MM_TO_INCHES;
    back_reading *= MM_TO_INCHES;
    left_reading *= MM_TO_INCHES;
    right_reading *= MM_TO_INCHES;
    
    // Validate readings (within sensor range)
    bool front_valid = front_reading > 0.39f && front_reading < MAX_LIDAR_RANGE; // >10mm
    bool back_valid = back_reading > 0.39f && back_reading < MAX_LIDAR_RANGE;
    bool left_valid = left_reading > 0.39f && left_reading < MAX_LIDAR_RANGE;
    bool right_valid = right_reading > 0.39f && right_reading < MAX_LIDAR_RANGE;
    
    int num_valid_sensors = (front_valid ? 1 : 0) + (back_valid ? 1 : 0) + 
                           (left_valid ? 1 : 0) + (right_valid ? 1 : 0);
    
    // Need at least 2 valid distance sensors for reliable position observation
    if (num_valid_sensors < 2) {
        return false; // Not enough valid sensors for position observation
    }
    
    // Storage for position estimates from each sensor
    std::vector<float> x_estimates, y_estimates;
    std::vector<float> x_weights, y_weights;
    
    float cos_theta = cos(theta_rad);
    float sin_theta = sin(theta_rad);
    
    // Compute position estimates from each valid sensor
    if (front_valid) {
        float offset_field_x = sensor_offset_front_x * cos_theta - sensor_offset_front_y * sin_theta;
        float offset_field_y = sensor_offset_front_x * sin_theta + sensor_offset_front_y * cos_theta;
        float estimated_y = FIELD_HEIGHT - front_reading - offset_field_y;
        
        y_estimates.push_back(estimated_y);
        y_weights.push_back(1.0f - front_reading / MAX_LIDAR_RANGE);
    }
    
    if (back_valid) {
        float offset_field_x = sensor_offset_back_x * cos_theta - sensor_offset_back_y * sin_theta;
        float offset_field_y = sensor_offset_back_x * sin_theta + sensor_offset_back_y * cos_theta;
        float estimated_y = back_reading - offset_field_y;
        
        y_estimates.push_back(estimated_y);
        y_weights.push_back(1.0f - back_reading / MAX_LIDAR_RANGE);
    }
    
    if (left_valid) {
        float offset_field_x = sensor_offset_left_x * cos_theta - sensor_offset_left_y * sin_theta;
        float offset_field_y = sensor_offset_left_x * sin_theta + sensor_offset_left_y * cos_theta;
        float estimated_x = left_reading - offset_field_x;
        
        x_estimates.push_back(estimated_x);
        x_weights.push_back(1.0f - left_reading / MAX_LIDAR_RANGE);
    }
    
    if (right_valid) {
        float offset_field_x = sensor_offset_right_x * cos_theta - sensor_offset_right_y * sin_theta;
        float offset_field_y = sensor_offset_right_x * sin_theta + sensor_offset_right_y * cos_theta;
        float estimated_x = FIELD_WIDTH - right_reading - offset_field_x;
        
        x_estimates.push_back(estimated_x);
        x_weights.push_back(1.0f - right_reading / MAX_LIDAR_RANGE);
    }
    
    // ========================================
    // STEP 3: Compute Weighted Average Position Observation
    // ========================================
    float observed_x = drive_x; // Default to current position
    float observed_y = drive_y;
    
    // Compute weighted average for X position
    if (!x_estimates.empty()) {
        float weighted_sum_x = 0, total_weight_x = 0;
        for (size_t i = 0; i < x_estimates.size(); i++) {
            weighted_sum_x += x_estimates[i] * x_weights[i];
            total_weight_x += x_weights[i];
        }
        if (total_weight_x > 0) {
            observed_x = weighted_sum_x / total_weight_x;
        }
    }
    
    // Compute weighted average for Y position  
    if (!y_estimates.empty()) {
        float weighted_sum_y = 0, total_weight_y = 0;
        for (size_t i = 0; i < y_estimates.size(); i++) {
            weighted_sum_y += y_estimates[i] * y_weights[i];
        }
        if (total_weight_y > 0) {
            observed_y = weighted_sum_y / total_weight_y;
        }
    }
    
    // ========================================
    // STEP 4: Validate Observation Quality
    // ========================================
    
    // Check if position observation is within field boundaries
    if (observed_x < 0 || observed_x > FIELD_WIDTH || 
        observed_y < 0 || observed_y > FIELD_HEIGHT) {
        return false; // Position observation is outside valid field
    }
    
    // Check for reasonable change from current estimate (detect outliers)
    State current_estimate = ukf->get_estimate();
    float position_change = sqrt(pow(observed_x - current_estimate.x, 2) + 
                               pow(observed_y - current_estimate.y, 2));
    
    // Reject observations that imply unrealistic jumps (>2 feet in 10ms)
    const float MAX_POSITION_JUMP = 24.0f; // inches
    if (position_change > MAX_POSITION_JUMP) {
        return false; // Observation implies unrealistic position change
    }
    
    // Check angular change
    float heading_change = fabs(observed_heading - current_estimate.heading * 180.0f / M_PI);
    while (heading_change > 180.0f) heading_change -= 360.0f; // Normalize
    while (heading_change < -180.0f) heading_change += 360.0f;
    
    // Reject observations that imply unrealistic angular jumps (>90 degrees in 10ms)
    const float MAX_HEADING_JUMP = 90.0f; // degrees
    if (fabs(heading_change) > MAX_HEADING_JUMP) {
        return false; // Observation implies unrealistic heading change
    }
    
    // ========================================
    // STEP 5: Store Valid Observation for UKF Correction
    // ========================================
    
    // Update the current position with the validated observation
    // This will be used by get_measurement() during UKF correct() step
    drive_x = observed_x;
    drive_y = observed_y;
    heading = observed_heading;
    
    return true; // Observation is valid and ready for UKF correction
}

Measurement OWDrivetrain::predict_measurement_from_state(const State& state) {
    // Measurement model h(x): Given a state estimate, predict what sensors should read
    // This is the mathematical function that maps state space to measurement space
    
    Measurement predicted;
    
    // Convert state heading from radians to match our coordinate system
    float theta_rad = state.heading; // State heading is already in radians
    float cos_theta = cos(theta_rad);
    float sin_theta = sin(theta_rad);
    
    // ========================================
    // PREDICT DISTANCE SENSOR READINGS
    // ========================================
    
    // Front sensor prediction (+Y direction when robot faces north)
    // Calculate sensor position in global frame
    float front_sensor_global_x = state.x + sensor_offset_front_x * cos_theta - sensor_offset_front_y * sin_theta;
    float front_sensor_global_y = state.y + sensor_offset_front_x * sin_theta + sensor_offset_front_y * cos_theta;
    
    // Calculate sensor orientation in global frame (front sensor points forward)
    float front_sensor_angle = theta_rad; // Front sensor aligned with robot
    
    // Distance to north wall (y = FIELD_HEIGHT) along sensor direction
    // Use safer threshold and check distance validity
    if (fabs(cos(front_sensor_angle)) > MIN_ANGLE_COS) { // cos(80°) ≈ 0.17
        float dist = (FIELD_HEIGHT - front_sensor_global_y) / cos(front_sensor_angle);
        if (dist > 0 && dist < MAX_LIDAR_RANGE) {
            predicted.f = dist;
        } else {
            predicted.f = MAX_LIDAR_RANGE;
        }
    } else {
        predicted.f = MAX_LIDAR_RANGE; // Sensor not pointing toward wall
    }
    
    // Back sensor prediction (-Y direction, 180° from front)
    float back_sensor_global_x = state.x + sensor_offset_back_x * cos_theta - sensor_offset_back_y * sin_theta;
    float back_sensor_global_y = state.y + sensor_offset_back_x * sin_theta + sensor_offset_back_y * cos_theta;
    
    float back_sensor_angle = theta_rad + PI; // Back sensor points backward
    
    // Distance to south wall (y = 0) along sensor direction  
    if (fabs(cos(back_sensor_angle)) > MIN_ANGLE_COS && cos(back_sensor_angle) < 0) {
        float dist = back_sensor_global_y / (-cos(back_sensor_angle));
        if (dist > 0 && dist < MAX_LIDAR_RANGE) {
            predicted.b = dist;
        } else {
            predicted.b = MAX_LIDAR_RANGE;
        }
    } else {
        predicted.b = MAX_LIDAR_RANGE;
    }
    
    // Left sensor prediction (-X direction, 90° left from front)
    float left_sensor_global_x = state.x + sensor_offset_left_x * cos_theta - sensor_offset_left_y * sin_theta;
    float left_sensor_global_y = state.y + sensor_offset_left_x * sin_theta + sensor_offset_left_y * cos_theta;
    
    float left_sensor_angle = theta_rad + PI/2.0f; // Left sensor points left
    
    // Distance to west wall (x = 0) along sensor direction
    if (fabs(sin(left_sensor_angle)) > MIN_ANGLE_COS && sin(left_sensor_angle) < 0) {
        float dist = left_sensor_global_x / (-sin(left_sensor_angle));
        if (dist > 0 && dist < MAX_LIDAR_RANGE) {
            predicted.l = dist;
        } else {
            predicted.l = MAX_LIDAR_RANGE;
        }
    } else {
        predicted.l = MAX_LIDAR_RANGE;
    }
    
    // Right sensor prediction (+X direction, 90° right from front)
    float right_sensor_global_x = state.x + sensor_offset_right_x * cos_theta - sensor_offset_right_y * sin_theta;
    float right_sensor_global_y = state.y + sensor_offset_right_x * sin_theta + sensor_offset_right_y * cos_theta;
    
    float right_sensor_angle = theta_rad - PI/2.0f; // Right sensor points right
    
    // Distance to east wall (x = FIELD_WIDTH) along sensor direction
    if (fabs(sin(right_sensor_angle)) > MIN_ANGLE_COS && sin(right_sensor_angle) > 0) {
        float dist = (FIELD_WIDTH - right_sensor_global_x) / sin(right_sensor_angle);
        if (dist > 0 && dist < MAX_LIDAR_RANGE) {
            predicted.r = dist;
        } else {
            predicted.r = MAX_LIDAR_RANGE;
        }
    } else {
        predicted.r = MAX_LIDAR_RANGE;
    }
    
    // ========================================
    // PREDICT OTHER SENSOR READINGS
    // ========================================
    
    // Rotation encoder: predict based on accumulated distance traveled
    // For a simplified model, we can predict the encoder reading based on current state
    // This assumes the encoder measures distance traveled by the tracking wheel
    
    // Get current encoder reading as baseline (this should be maintained as part of state)
    // For now, predict no change from current reading (stationary prediction)
    // TODO: Add encoder position to State struct for proper tracking
    predicted.rotation = 0; // Placeholder - needs proper encoder position tracking
    
    // Heading: predict IMU reading (should match state heading)
    predicted.heading = state.heading * 180.0f / PI; // Convert radians to degrees
    
    // ========================================
    // BOUNDS CHECKING
    // ========================================
    
    // Clamp distance predictions to valid sensor range
    predicted.f = std::max(0.39f, std::min(MAX_LIDAR_RANGE, predicted.f));
    predicted.b = std::max(0.39f, std::min(MAX_LIDAR_RANGE, predicted.b));
    predicted.l = std::max(0.39f, std::min(MAX_LIDAR_RANGE, predicted.l));
    predicted.r = std::max(0.39f, std::min(MAX_LIDAR_RANGE, predicted.r));
    
    return predicted;
}

UKF::UKF(OWDrivetrain& drivetrain, State state, State mean, State sd, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> p_noise, std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> m_noise, float alpha, float beta, float kappa, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> se_cov):
    drivetrain(drivetrain),
    current(state),
    mean(mean),
    sd(sd),
    process_noise(p_noise),
    measurement_noise(m_noise),
    alpha(alpha),
    beta(beta),
    kappa(kappa),
    se_cov(se_cov) // array with all 0
{
    // REMOVED: scaling_factor calculation - now calculated locally as lambda
    
    float current_state[7] = {current.x, current.y, current.heading, current.lv, current.av, current.imu_bias, current.encoder_bias};

    predicted_measurement = {
        0,0,0,0,0,0
    };

    predicted_state = {
        0,0,0,0,0,0,0
    };

    se_cov[0][0] = pow(mean.x, 2);
    se_cov[1][1] = pow(mean.y, 2);
    se_cov[2][2] = pow(mean.heading, 2);
    se_cov[3][3] = pow(mean.lv, 2);
    se_cov[4][4] = pow(mean.av, 2);
    se_cov[5][5] = pow(mean.imu_bias, 2);
    se_cov[6][6] = pow(mean.encoder_bias, 2);
};

State UKF::get_estimate() {
    return current;
}

void UKF::set_num_sigma_points() {
    // Number of sigma points is already defined as NUM_SIGMA_POINTS = 2n+1 where n is STATE_DIMENSIONS
    // This function is now obsolete since we use constexpr NUM_SIGMA_POINTS = 15
}

State& State::operator+=(const State& rhs) {
    this->x += rhs.x;
    this->y += rhs.y;
    this->heading += rhs.heading;
    this->lv += rhs.lv;
    this->av += rhs.av;
    this->imu_bias += rhs.imu_bias;
    this->encoder_bias+= rhs.encoder_bias;
    return *this;
}

State operator+(const State& lhs, const State& rhs) {
    State result = lhs;
    result += rhs;
    return result;
}

std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> UKF::matrix_sqrt(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix){
    // Cholesky decomposition: finds L such that L * L^T = matrix
    // This is the mathematically correct "square root" for UKF sigma points
    
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> L;
    
    // Initialize to zero
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            L[i][j] = 0.0f;
        }
    }
    
    // Cholesky decomposition algorithm
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0.0f;
            
            if (j == i) { // Diagonal elements
                for (int k = 0; k < j; k++) {
                    sum += L[j][k] * L[j][k];
                }
                float val = matrix[j][j] - sum;
                if (val <= 0.0f) {
                    // Matrix is not positive definite - add regularization
                    val = 1e-6f;
                }
                L[j][j] = sqrt(val);
            } else { // Off-diagonal elements
                for (int k = 0; k < j; k++) {
                    sum += L[i][k] * L[j][k];
                }
                if (L[j][j] != 0.0f) {
                    L[i][j] = (matrix[i][j] - sum) / L[j][j];
                } else {
                    L[i][j] = 0.0f;
                }
            }
        }
    }
    
    return L;
}

std::array<float, STATE_DIMENSIONS> UKF::col_sqrt(std::array<float, STATE_DIMENSIONS> col) { 
    // This function is now obsolete since we use proper Cholesky decomposition
    // in set_sigma_points(). Keeping for backward compatibility but not used.
    std::array<float, STATE_DIMENSIONS> result;
    
    for (int i=0;i<STATE_DIMENSIONS;i++) {
        result[i]=sqrt(col[i]); // Removed scaling_factor reference
    }

    return result;
}

State UKF::list_to_state(std::array<float, STATE_DIMENSIONS> col) {
    State result;
    result.x = col[0];
    result.y = col[1];
    result.heading = col[2];
    result.lv = col[3];
    result.av = col[4];
    result.imu_bias = col[5];
    result.encoder_bias = col[6];
    return result;
}

void UKF::set_sigma_points() {
    // Set sigma points using CORRECT UKF sigma point generation
    // CRITICAL: Take Cholesky FIRST, then scale the result
    
    // First sigma point is the mean state
    points_sigma[0] = current; 
    
    // Calculate lambda for scaling
    float lambda = alpha * alpha * (STATE_DIMENSIONS + kappa) - STATE_DIMENSIONS;
    float scale = sqrt(STATE_DIMENSIONS + lambda);
    
    // STEP 1: Get Cholesky decomposition of covariance FIRST
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> L = 
        matrix_sqrt(se_cov);
    
    // STEP 2: THEN scale the Cholesky factor (not the covariance!)
    // Mathematical property: chol(c*P) = sqrt(c) * chol(P)
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> scaled_L = 
        multiply_const_mat(L, scale);
    
    // Generate positive sigma points (indices 1 to STATE_DIMENSIONS)
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        std::array<float, STATE_DIMENSIONS> col;
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            col[j] = scaled_L[j][i];  // Use scaled_L, not L!
        }
        State offset = list_to_state(col);
        points_sigma[i + 1] = current + offset;
    }
    
    // Generate negative sigma points (indices STATE_DIMENSIONS+1 to 2*STATE_DIMENSIONS)
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        std::array<float, STATE_DIMENSIONS> col;
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            col[j] = -scaled_L[j][i];  // Use scaled_L, not L!
        }
        State offset = list_to_state(col);
        points_sigma[STATE_DIMENSIONS + i + 1] = current + offset;
    }
} 

State& State::operator*(const float factor) {
    this->x *= factor;
    this->y *= factor;
    this->heading *= factor;
    this->lv *= factor;
    this->av *= factor;
    this->imu_bias *= factor;
    this->encoder_bias *= factor;
    return *this;  // CRITICAL: Missing return statement
}

void UKF::compute_predicted_measurements() {
    /*
    Compute predicted measurement for each sigma point using measurement model h(x)
    This predicts what sensors SHOULD read given each sigma point state
    */
    
    // CRITICAL FIX: Loop should be 0 to (2*STATE_DIMENSIONS+1), not 2*(2*STATE_DIMENSIONS+1)
    // We have 15 sigma points, not 30!
    for (int i = 0; i < 2*STATE_DIMENSIONS + 1; i++) {
        measurement_sigma[i] = drivetrain.predict_measurement_from_state(points_sigma[i]);
    }
}

Measurement& Measurement::operator+=(const Measurement& rhs) {
    this->l += rhs.l;
    this->r += rhs.r;
    this->f += rhs.f;
    this->b += rhs.b;
    this->rotation += rhs.rotation;
    this->heading += rhs.heading;
    return *this;
}

Measurement operator+(const Measurement& lhs, const Measurement& rhs) {
    Measurement result = lhs;
    result += rhs;
    return result;
}

Measurement& Measurement::operator-=(const Measurement& rhs) {
    this->l -= rhs.l;
    this->r -= rhs.r;
    this->f -= rhs.f;
    this->b -= rhs.b;
    this->rotation -= rhs.rotation;
    this->heading -= rhs.heading;
    return *this;
}

Measurement operator-(const Measurement& lhs, const Measurement& rhs) {
    Measurement result = lhs;
    result -= rhs;
    return result;
}

Measurement& Measurement::operator*(const float& rhs) {
    this->l *= rhs;
    this->r *= rhs;
    this->f *= rhs;
    this->b *= rhs;
    this->rotation *= rhs;
    this->heading *= rhs;
    return *this;
}

Measurement& Measurement::operator*(const Measurement& rhs) {
    this->l *= rhs.l;
    this->r *= rhs.r;
    this->f *= rhs.f;
    this->b *= rhs.b;
    this->rotation *= rhs.rotation;
    this->heading *= rhs.heading;
    return *this;
}

void UKF::combine_predicted_measurements() {
    /*
    Combine predicted measurements using correct UKF weights
    */

    // CRITICAL FIX: Use standard UKF weight formulas
    float lambda = alpha * alpha * (STATE_DIMENSIONS + kappa) - STATE_DIMENSIONS;
    
    // Mean weight for first sigma point (mean)
    mean_weight[0] = lambda / (STATE_DIMENSIONS + lambda);
    
    // Weights for remaining sigma points
    float remaining_weight = 1.0f / (2.0f * (STATE_DIMENSIONS + lambda));
    for (int i = 1; i < 2*STATE_DIMENSIONS + 1; i++) {
        mean_weight[i] = remaining_weight;
    }

    // Reset predicted_measurement to zero first
    predicted_measurement = {0,0,0,0,0,0};

    // FIXED: Use correct loop bounds
    for (int i = 0; i < 2*STATE_DIMENSIONS + 1; i++) {
        predicted_measurement += measurement_sigma[i] * mean_weight[i];
    }
}

void UKF::est_cov_predicted_measurements() {
    /*
    Estimate covariance of predicted measurements using correct weights
    */

    // CRITICAL FIX: Use standard UKF covariance weight formulas
    float lambda = alpha * alpha * (STATE_DIMENSIONS + kappa) - STATE_DIMENSIONS;
    
    // Covariance weight for first sigma point
    cov_weight[0] = lambda / (STATE_DIMENSIONS + lambda) + (1 - alpha*alpha + beta);
    
    // Covariance weights for remaining sigma points  
    float remaining_cov_weight = 1.0f / (2.0f * (STATE_DIMENSIONS + lambda));
    for (int i = 1; i < 2*STATE_DIMENSIONS + 1; i++) {
        cov_weight[i] = remaining_cov_weight;
    }

    // FIXED: Use correct loop bounds
    for (int i = 0; i < 2*STATE_DIMENSIONS + 1; i++) {

        measurement_vals[0][i] = measurement_sigma[i].l;
        measurement_vals[1][i] = measurement_sigma[i].r;
        measurement_vals[2][i] = measurement_sigma[i].f;
        measurement_vals[3][i] = measurement_sigma[i].b;
        measurement_vals[4][i] = measurement_sigma[i].rotation;
        measurement_vals[5][i] = measurement_sigma[i].heading;
    }

    // Store predicted measurement for comparison
    int mean_idx = 2*STATE_DIMENSIONS + 1;  // Index for storing mean
    measurement_vals[0][mean_idx] = predicted_measurement.l;
    measurement_vals[1][mean_idx] = predicted_measurement.r;
    measurement_vals[2][mean_idx] = predicted_measurement.f;
    measurement_vals[3][mean_idx] = predicted_measurement.b;
    measurement_vals[4][mean_idx] = predicted_measurement.rotation;
    measurement_vals[5][mean_idx] = predicted_measurement.heading;

    for (int i=0;i<MEASUREMENT_DIMENSIONS;i++) {
        for (int j=0;j<MEASUREMENT_DIMENSIONS;j++) {
            measurement_cov[i][j] = 0;
            // FIXED: Use correct loop bounds for sigma points
            for (int k=0; k < 2*STATE_DIMENSIONS + 1; k++) {
                measurement_cov[i][j] += cov_weight[k]*(measurement_vals[i][k]-measurement_vals[i][mean_idx]) * (measurement_vals[j][k]-measurement_vals[j][mean_idx]);
            }
            // Add measurement noise R to the measurement covariance
            measurement_cov[i][j] += measurement_noise[i][j];
        }
    }
}

void UKF::state_to_list() {
    // FIXED: Correct loop bounds and remove redundant weight calculation
    for (int i = 0; i < 2*STATE_DIMENSIONS + 1; i++) {
        state_vals[0][i] = points_sigma[i].x;
        state_vals[1][i] = points_sigma[i].y;
        state_vals[2][i] = points_sigma[i].heading;
        state_vals[3][i] = points_sigma[i].lv;
        state_vals[4][i] = points_sigma[i].av;
        state_vals[5][i] = points_sigma[i].imu_bias;
        state_vals[6][i] = points_sigma[i].encoder_bias;
    }

    // Store predicted state for comparison  
    int mean_idx = 2*STATE_DIMENSIONS + 1;
    state_vals[0][mean_idx] = predicted_state.x;
    state_vals[1][mean_idx] = predicted_state.y;
    state_vals[2][mean_idx] = predicted_state.heading;
    state_vals[3][mean_idx] = predicted_state.lv;
    state_vals[4][mean_idx] = predicted_state.av;
    state_vals[5][mean_idx] = predicted_state.imu_bias;
    state_vals[6][mean_idx] = predicted_state.encoder_bias;
}

void UKF::est_cross_cov() {
    /*
    Cross-covariance between state and measurement
    P_xy = Σ W_c[i] * (X[i] - x̄) * (Z[i] - z̄)^T
    */
    
    // Populate state and measurement arrays
    state_to_list();
    
    // Initialize cross-covariance matrix
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
            se_me_cov[i][j] = 0;
            // FIXED: Correct loop bounds for sigma points
            int mean_idx = 2*STATE_DIMENSIONS + 1;
            for (int k = 0; k < 2*STATE_DIMENSIONS + 1; k++) {
                float state_diff = state_vals[i][k] - state_vals[i][mean_idx];
                float measurement_diff = measurement_vals[j][k] - measurement_vals[j][mean_idx];
                
                // Special handling for heading differences (wrap angles)
                if (i == 2) { // heading state
                    state_diff = normalize_angle_symmetric(state_diff);
                }
                if (j == 5) { // heading measurement
                    measurement_diff = normalize_angle_symmetric(measurement_diff);
                }
                
                se_me_cov[i][j] += cov_weight[k] * state_diff * measurement_diff;
            }
        }
    }
}

void UKF::obtain_est() {
    /*
    Obtain the estimated state and state estimation error covariance at time step k.
    */
    
    // Get actual measurement from drivetrain (real sensor readings)
    Measurement actual_measurement = drivetrain.get_measurement(current);
    
    // Calculate measurement residual with proper angle handling
    Measurement residual;
    residual.l = actual_measurement.l - predicted_measurement.l;
    residual.r = actual_measurement.r - predicted_measurement.r;
    residual.f = actual_measurement.f - predicted_measurement.f;
    residual.b = actual_measurement.b - predicted_measurement.b;
    residual.rotation = actual_measurement.rotation - predicted_measurement.rotation;
    
    // CRITICAL: Normalize heading residual to handle angle wrapping
    // Convert both to radians for proper normalization
    float actual_heading_rad = actual_measurement.heading * M_PI / 180.0f;
    float predicted_heading_rad = predicted_measurement.heading * M_PI / 180.0f;
    residual.heading = normalize_angle_symmetric(actual_heading_rad - predicted_heading_rad) * 180.0f / M_PI;
    
    // Convert residual to vector for matrix operations
    float residual_vec[MEASUREMENT_DIMENSIONS] = {
        residual.l, residual.r, residual.f, residual.b, residual.rotation, residual.heading
    };
    
    // Calculate Kalman gain: K = P_xy * inv(P_yy)
    // First compute inverse of measurement covariance matrix
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> inv_measurement_cov = 
        inverse_mat_66(measurement_cov);
    
    // Multiply cross-covariance by inverse measurement covariance to get Kalman gain
    std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> kalman_gain = 
        multiply_mat_76_66(se_me_cov, inv_measurement_cov);
    
    // Update state estimate: x = x_predicted + K * (z - h(x_predicted))
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        float correction = 0;
        for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
            correction += kalman_gain[i][j] * residual_vec[j];
        }
        
        switch(i) {
            case 0: current.x = predicted_state.x + correction; break;
            case 1: current.y = predicted_state.y + correction; break;
            case 2: current.heading = predicted_state.heading + correction; break;
            case 3: current.lv = predicted_state.lv + correction; break;
            case 4: current.av = predicted_state.av + correction; break;
            case 5: current.imu_bias = predicted_state.imu_bias + correction; break;
            case 6: current.encoder_bias = predicted_state.encoder_bias + correction; break;
        }
    }
    
    // Update state estimation error covariance: P = P_predicted - K * P_yy * K^T
    std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> k_pyy = 
        multiply_mat_76_66(kalman_gain, measurement_cov);
    
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> k_pyy_kt = 
        multiply_mat_76_67(k_pyy, transpose_mat_76(kalman_gain));
    
    se_cov = subtract_mat_77(predicted_se_cov, k_pyy_kt);
    
    // Validate and clamp the updated state
    validate_and_clamp_state();
}

void UKF::validate_and_clamp_state() {
    // Clamp position to field bounds with small margin
    const float MARGIN = 1.0f; // 1 inch margin from field edges
    current.x = std::max(MARGIN, std::min(FIELD_WIDTH - MARGIN, current.x));
    current.y = std::max(MARGIN, std::min(FIELD_HEIGHT - MARGIN, current.y));
    
    // Normalize heading to [-π, π]
    current.heading = normalize_angle_symmetric(current.heading);
    
    // Clamp velocities to reasonable bounds
    const float MAX_LINEAR_VEL = 100.0f;  // inches/sec
    const float MAX_ANGULAR_VEL = 2.0f * M_PI;  // rad/sec
    current.lv = std::max(-MAX_LINEAR_VEL, std::min(MAX_LINEAR_VEL, current.lv));
    current.av = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, current.av));
    
    // Clamp biases to reasonable ranges
    const float MAX_IMU_BIAS = 0.1f;  // rad/sec
    const float MAX_ENCODER_BIAS = 10.0f;  // inches/sec
    current.imu_bias = std::max(-MAX_IMU_BIAS, std::min(MAX_IMU_BIAS, current.imu_bias));
    current.encoder_bias = std::max(-MAX_ENCODER_BIAS, std::min(MAX_ENCODER_BIAS, current.encoder_bias));
    
    // Check for NaN or inf values
    if (std::isnan(current.x) || std::isnan(current.y) || std::isnan(current.heading) ||
        std::isinf(current.x) || std::isinf(current.y) || std::isinf(current.heading)) {
        
        // Reset to a safe default state
        current.x = FIELD_WIDTH / 2.0f;
        current.y = FIELD_HEIGHT / 2.0f;
        current.heading = 0.0f;
        current.lv = 0.0f;
        current.av = 0.0f;
        current.imu_bias = 0.0f;
        current.encoder_bias = 0.0f;
        
        // Also reset covariance to prevent further issues
        for (int i = 0; i < STATE_DIMENSIONS; i++) {
            for (int j = 0; j < STATE_DIMENSIONS; j++) {
                se_cov[i][j] = (i == j) ? 1.0f : 0.0f; // Identity matrix
            }
        }
    }
}

void UKF::enforce_covariance_symmetry() {
    // Ensure covariance matrix is symmetric by averaging off-diagonal elements
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = i + 1; j < STATE_DIMENSIONS; j++) {
            float avg = (se_cov[i][j] + se_cov[j][i]) / 2.0f;
            se_cov[i][j] = avg;
            se_cov[j][i] = avg;
        }
        // Ensure diagonal elements are positive (add small regularization if needed)
        if (se_cov[i][i] <= 0.0f) {
            se_cov[i][i] = 1e-6f;
        }
    }
}

// DELETED: predict_set_sigma_points() was mathematically wrong and not needed
// - It tried to take Cholesky of negative matrices (undefined!)
// - It used wrong covariance matrix (se_cov instead of predicted_se_cov)
// - UKF doesn't need separate sigma point generation in predict step 

State UKF::state_transition(const State& sigma_point, double left_voltage, double right_voltage, double dt) {
    State next_state;
    
    // Convert voltages to wheel velocities (simplified model)
    // TODO: Make these configurable parameters instead of hardcoded constants
    const double voltage_to_velocity = 0.1; // This should be calibrated based on your robot
    const double wheel_radius = 2.75; // inches, adjust based on your robot
    const double track_width = 16; // inches, distance between wheels, adjust based on your robot
    
    double left_wheel_velocity = left_voltage * voltage_to_velocity;
    double right_wheel_velocity = right_voltage * voltage_to_velocity;
    
    // Differential drive kinematics
    double linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0;
    double angular_velocity = (right_wheel_velocity - left_wheel_velocity) / track_width;
    
    // Add biases
    linear_velocity += sigma_point.encoder_bias;
    angular_velocity += sigma_point.imu_bias;
    
    // State propagation using kinematic model
    next_state.x = sigma_point.x + linear_velocity * cos(sigma_point.heading) * dt;
    next_state.y = sigma_point.y + linear_velocity * sin(sigma_point.heading) * dt;
    next_state.heading = normalize_angle_symmetric(sigma_point.heading + angular_velocity * dt);
    
    // Velocity states with simple dynamics
    next_state.lv = linear_velocity; // Could add acceleration model here
    next_state.av = angular_velocity; // Could add acceleration model here
    
    // Bias states (assumed to be random walk)
    next_state.imu_bias = sigma_point.imu_bias;
    next_state.encoder_bias = sigma_point.encoder_bias;
    
    return next_state;
}

void UKF::apply_state_transition(double left_voltage, double right_voltage, double dt) {
    // Apply state transition function to all sigma points to get predicted sigma points
    // FIXED: Correct loop bounds for sigma points
    for (int i = 0; i < 2*STATE_DIMENSIONS + 1; i++) {
        predicted_points_sigma[i] = state_transition(points_sigma[i], left_voltage, right_voltage, dt);
    }
}

void UKF::compute_predicted_states() {
    next = {0,0,0,0,0,0,0};
    // FIXED: Correct loop bounds and normalize heading
    for (int i = 0; i < 2*STATE_DIMENSIONS + 1; i++) {
        next += predicted_points_sigma[i]*mean_weight[i];
    }
    // Normalize the predicted heading
    next.heading = normalize_angle_symmetric(next.heading);
}
void UKF::compute_predicted_cov() {
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            predicted_se_cov[i][j] = 0;
        }
    }
    // FIXED: Correct loop bounds and angle difference handling
    for (int i = 0; i < 2*STATE_DIMENSIONS + 1; i++) {
        State diff;
        diff.x = predicted_points_sigma[i].x - next.x;
        diff.y = predicted_points_sigma[i].y - next.y;
        diff.heading = normalize_angle_symmetric(predicted_points_sigma[i].heading - next.heading);
        diff.lv = predicted_points_sigma[i].lv - next.lv;
        diff.av = predicted_points_sigma[i].av - next.av;
        diff.imu_bias = predicted_points_sigma[i].imu_bias - next.imu_bias;
        diff.encoder_bias = predicted_points_sigma[i].encoder_bias - next.encoder_bias;
        
        float state_vec[STATE_DIMENSIONS] = {diff.x, diff.y, diff.heading, diff.lv, diff.av, diff.imu_bias, diff.encoder_bias};
        
        for (int row = 0; row < STATE_DIMENSIONS; row++) {
            for (int col = 0; col < STATE_DIMENSIONS; col++) {
                predicted_se_cov[row][col] += cov_weight[i] * state_vec[row] * state_vec[col];
            }
        }
    }
    
    // Add process noise Q to the predicted covariance
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            predicted_se_cov[i][j] += process_noise[i][j];
        }
    }
    
    // Ensure predicted covariance stays symmetric and positive definite
    enforce_covariance_symmetry();
}

void UKF::correct() {
    // update state & state estimation error covariance. collect data from drivetrain
    // update CURRENT state and state estimation error covariance. For each sigma point at time step
        // call set sigma points
        // call set offset sigma points using sqrt(scaling_factor * P)

    set_sigma_points();

    // use drivetrain to compute predicted measurement for each sigma point
    compute_predicted_measurements();

    // combine predicted measurements using summation of weight_mean * measurement
    combine_predicted_measurements();
    
    // estimate covariance of predicted measurement. add R[k] to account for additive measurement noise
    est_cov_predicted_measurements();

    // estimate cross covariance between x (state) and y (measurement)
    est_cross_cov();

    // calculate kalman gain, estimate state, state estimation error covariance matrix
    obtain_est();
}

void UKF::predict(double left_voltage, double right_voltage, double dt) {
    // CORRECT UKF PREDICT FLOW:
    // 1. Generate sigma points around CURRENT state and covariance
    // 2. Propagate sigma points through motion model
    // 3. Compute predicted mean and covariance from propagated points
    
    // STEP 1: Generate sigma points around current state
    set_sigma_points();  // Uses current and se_cov
    
    // STEP 2: Apply state transition to propagate sigma points forward
    apply_state_transition(left_voltage, right_voltage, dt);

    // STEP 3: Compute predicted state mean from propagated sigma points
    compute_predicted_states();
    
    // STEP 4: Compute predicted covariance from propagated sigma points
    compute_predicted_cov();
    
    // STEP 5: Update current state to predicted result
    current = next;
    se_cov = predicted_se_cov;
    
    // Increment time step for predict cycle
    time_step++;
} 

void UKF::run(double left_voltage, double right_voltage, double dt) {
    // Legacy method - now prefer manual predict/correct calls for proper ordering
    // This maintains backward compatibility but is not the recommended approach
    this->correct();
    this->predict(left_voltage, right_voltage, dt);
    time_step++;
}

bool OWDrivetrain::is_front_sensor_within_distance(float target_distance, float tolerance) {
    const float MM_TO_INCHES = 1.0f / 25.4f;
    float current_distance = front_dist->get() * MM_TO_INCHES; // Convert to inches
    
    if (current_distance < 0) {
        return false;
    }
    
    return fabs(current_distance - target_distance) <= tolerance;
}