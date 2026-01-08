#include "lib.h"
#include "api.h"
#include "util.h"
#include <cmath>
#include <vector>
#include <algorithm>

// ------------------------LIDAR------------------------------------------------------------------------ 
// Constants for LiDAR localization
const float FIELD_SIZE = 3658.0f; // 12 feet in mm
const float MAX_LIDAR_RANGE = 2000.0f;
const float ASSOCIATION_THRESHOLD = 300.0f; // mm
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

    this->integral += error; // integral

    // when error reaches 0, set integral to 0
    if (fabs(this->integral) < this->settle_error) {
        this->integral = 0;
    }

    // handle integral wind-up: 1) limit range of integral build up
    if (fabs(error) > this->integral_threshold) {
        this->integral = 0;
    }

    // rate of change of error
    float derivative = error - this->prev_error;
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
    std::vector<Point> line = get_line();

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

Point PurePursuit::set_goal_point(Point current_pos) {
    std::vector<Point> intersections = get_intersection(current_pos, waypoints[waypoint_index-1], waypoints[waypoint_index], 1);
    int size = intersections.size();

    if (size == 2) {
        if (distance(intersections[0], waypoints[waypoint_index]) < distance(intersections[1], waypoints[waypoint_index])) {
            // closer to waypoint
            if (distance(current_pos, waypoints[waypoint_index]) < distance(intersections[0], waypoints[waypoint_index])) {
                if (!(waypoint_index >= waypoints.size())) {
                    waypoint_index++;
                }
            } else {
                goal_point = intersections[0];
            }
        } else {
            if (distance(current_pos, waypoints[waypoint_index]) < distance(intersections[1], waypoints[waypoint_index])) {
                if (!(waypoint_index >= waypoints.size())) {
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
    float turn_error = atan2(dy,dx); // represents angle between heading vector & look-ahead vector
    // float curvature = 2*dx/pow(lookahead_distance, 2);
    return {linear_error, turn_error};
}

bool PurePursuit::get_settled(Point current_pos) {
    if (pow((current_pos.x - goal_point.x), 2) + pow((current_pos.y - goal_point.y), 2) <= settle_radius) {
        return true;
    }
    return false;
}

OWDrivetrain::OWDrivetrain(std::vector<std::int8_t> left_motor_ports, std::vector<std::int8_t> right_motor_ports, PID linear_pid, PID turn_pid, int motor_speed, int imu_port, int imu2_port, int left_tracker_port, float track_width, float left_tracker_offset, int left_dist_port, int right_dist_port, int front_dist_port, int back_dist_port):
    left_motors(left_motor_ports),
    right_motors(right_motor_ports),
    linear_pid(linear_pid),
    turn_pid(turn_pid),
    motor_speed(motor_speed),
    imu(imu_port),
    imu2(imu2_port),
    left_tracker(left_tracker_port),
    track_width(track_width),
    left_tracker_offset(left_tracker_offset),
    left_dist(left_dist_port),
    right_dist(right_dist_port),
    front_dist(front_dist_port),
    back_dist(back_dist_port)
{

};

Point OWDrivetrain::dist_est_pos() {
    // Define LiDAR configurations with physical offsets (you'll need to set these values)
    const float FRONT_OFFSET_X = 0.0f;   // Set actual front LiDAR X offset 
    const float FRONT_OFFSET_Y = 100.0f; // Set actual front LiDAR Y offset
    const float RIGHT_OFFSET_X = 100.0f; // Set actual right LiDAR X offset  
    const float RIGHT_OFFSET_Y = 0.0f;   // Set actual right LiDAR Y offset
    const float BACK_OFFSET_X = 0.0f;    // Set actual back LiDAR X offset
    const float BACK_OFFSET_Y = -100.0f; // Set actual back LiDAR Y offset
    const float LEFT_OFFSET_X = -100.0f; // Set actual left LiDAR X offset
    const float LEFT_OFFSET_Y = 0.0f;    // Set actual left LiDAR Y offset
    
    LidarConfig front_config = {FRONT_OFFSET_X, FRONT_OFFSET_Y, 0.0f};           // 0° (forward)
    LidarConfig right_config = {RIGHT_OFFSET_X, RIGHT_OFFSET_Y, -PI / 2.0f};     // -90° (right)
    LidarConfig back_config = {BACK_OFFSET_X, BACK_OFFSET_Y, PI};                // 180° (backward)
    LidarConfig left_config = {LEFT_OFFSET_X, LEFT_OFFSET_Y, PI / 2.0f};         // 90° (left)
    
    // Get current pose estimate (using current position and heading)
    float x_est = drive_x;
    float y_est = drive_y;
    float theta_est = heading * PI / 180.0f; // Convert degrees to radians
    
    // Collect LiDAR samples
    float l = left_dist.get();   // Left LiDAR distance in mm
    float r = right_dist.get();  // Right LiDAR distance in mm
    float f = front_dist.get();  // Front LiDAR distance in mm
    float b = back_dist.get();   // Back LiDAR distance in mm
    
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
    /*
    */

    left_pos = left_tracker.get_position();

    float distance = left_pos - prev_left_pos;

    prev_left_pos = left_pos;

    float convert_heading = -heading;

    return {drive_y + cos(convert_heading)*distance, drive_y + sin(convert_heading)*distance};
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
    Point lidar_pos = dist_est_pos();
    Point tracker_pos = tracker_est_pos();
    
    // Get raw sensor readings
    float l = left_dist.get();
    float r = right_dist.get(); 
    float f = front_dist.get();
    float b = back_dist.get();
    
    // Display current robot state
    pros::lcd::print(0, "=== LiDAR Debug ===");
    pros::lcd::print(1, "Current: X=%.1f Y=%.1f H=%.1f", current_pos.x, current_pos.y, heading);
    pros::lcd::print(2, "LiDAR:   X=%.1f Y=%.1f", lidar_pos.x, lidar_pos.y);
    pros::lcd::print(3, "Tracker: X=%.1f Y=%.1f", tracker_pos.x, tracker_pos.y);
    pros::lcd::print(4, "Distances: L=%.0f R=%.0f", l, r);
    pros::lcd::print(5, "           F=%.0f B=%.0f", f, b);
    
    // Calculate differences
    float lidar_error_x = lidar_pos.x - current_pos.x;
    float lidar_error_y = lidar_pos.y - current_pos.y;
    float tracker_error_x = tracker_pos.x - current_pos.x;
    float tracker_error_y = tracker_pos.y - current_pos.y;
    
    pros::lcd::print(6, "LiDAR Err: X=%.1f Y=%.1f", lidar_error_x, lidar_error_y);
    pros::lcd::print(7, "Track Err: X=%.1f Y=%.1f", tracker_error_x, tracker_error_y);
}

void OWDrivetrain::update_position() {
    heading = (min_angle(imu.get_heading()) + min_angle(imu2.get_heading()))/2;

    Point dist = dist_est_pos();
    Point tracker = tracker_est_pos();
    
    // Validate distance sensor position
    bool dist_valid = (dist.x != 0 || dist.y != 0) && 
                      !std::isnan(dist.x) && !std::isnan(dist.y) && 
                      std::isfinite(dist.x) && std::isfinite(dist.y);
    
    // Validate tracker position  
    bool tracker_valid = !std::isnan(tracker.x) && !std::isnan(tracker.y) && 
                         std::isfinite(tracker.x) && std::isfinite(tracker.y);
    
    // Sensor fusion with fallbacks
    if (dist_valid && tracker_valid) {
        // Both valid - use weighted average
        drive_x = 0.8*dist.x + 0.2*tracker.x;
        drive_y = 0.8*dist.y + 0.2*tracker.y;
    } else if (tracker_valid) {
        // Only tracker valid
        drive_x = tracker.x;
        drive_y = tracker.y;
    } else if (dist_valid) {
        // Only distance valid
        drive_x = dist.x;
        drive_y = dist.y;
    } else {
        // Neither valid - keep previous position
        drive_x = prev_drive_x;
        drive_y = prev_drive_y;
    }

    prev_drive_x = drive_x;
    prev_drive_y = drive_y;
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
    /* from input predicted state estimate, 
    apply measurement model to get measurement for each sigma point. 
    This function maps the state space to measurement space.
    */
    
    Measurement measurement;
    
    // Get current sensor readings
    measurement.l = left_dist.get();    // Left distance sensor
    measurement.r = right_dist.get();   // Right distance sensor  
    measurement.f = front_dist.get();   // Front distance sensor
    measurement.b = back_dist.get();    // Back distance sensor
    measurement.rotation = left_tracker.get_position(); // Rotation encoder
    measurement.heading = heading;      // IMU heading
    
    return measurement;
}

std::vector<Measurement> OWDrivetrain::sample_all_sensors(int num_samples, int delay_ms) {
    std::vector<Measurement> samples;
    samples.reserve(num_samples);
    
    for (int i = 0; i < num_samples; i++) {
        Measurement sample;
        
        sample.l = left_dist.get();
        sample.r = right_dist.get();
        sample.f = front_dist.get();
        sample.b = back_dist.get();
        sample.rotation = left_tracker.get_position();
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

UKF::UKF(OWDrivetrain& drivetrain, State state, State mean, State sd, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> p_noise, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> m_noise, float alpha, float beta, float kappa, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> se_cov):
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
    scaling_factor = pow(alpha, 2) * (num_states + kappa);
    
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

inline State operator+(const State& lhs, const State& rhs) {
    State result = lhs;
    result += rhs;
    return result;
}

std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> UKF::matrix_sqrt(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix){
    float trace = trace_mat_77(matrix); // Trace Matrix
    float det = det_mat_77(matrix); // Determinant 
    // Identity matrix

    return multiply_const_mat(add_mat(matrix, multiply_const_mat(identity_mat, sqrt(det))),(1/sqrt(trace+2*sqrt(det))));
}

std::array<float, STATE_DIMENSIONS> UKF::col_sqrt(std::array<float, STATE_DIMENSIONS> col) { // with scaling factor
    std::array<float, STATE_DIMENSIONS> result;
    
    for (int i=0;i<STATE_DIMENSIONS;i++) {
        result[i]=sqrt(col[i] * scaling_factor);
    }

    return result;
}

State list_to_state(std::array<float, STATE_DIMENSIONS> col) {
    return {
        x: col[0],
        y: col[1],
        heading: col[2],
        lv: col[3],
        av: col[4],
        imu_bias: col[5],
        encoder_bias: col[6]
    };
}

void UKF::set_sigma_points() {
    // Set sigma points using root cP, c = alpha^2 (M + kappa); M -> number of states/sigma points
    points_sigma[0] = current; 

    for (int i=0;i<STATE_DIMENSIONS;i++) {
        std::array<float, STATE_DIMENSIONS> col = UKF::matrix_sqrt(multiply_const_mat(se_cov, scaling_factor))[i];
        offset_sigma[i] = list_to_state(col);

    }

    for (int i=0;i<STATE_DIMENSIONS;i++) {
        std::array<float, STATE_DIMENSIONS> col = UKF::matrix_sqrt(multiply_const_mat(se_cov, -scaling_factor))[i];
        offset_sigma[i+STATE_DIMENSIONS] = list_to_state(col);
    }

    for (int i=0;i<2*num_states;i++) {
        points_sigma[i] = current + offset_sigma[i];
    }
} 

State& State::operator*(const float factor) {
    this->x *= factor;
    this->y *= factor;
    this->heading *= factor;
    this->lv *= factor;
    this->av *= factor;
    this->imu_bias *= factor;
    this->encoder_bias*= factor;
}

void UKF::compute_predicted_measurements() {
    /*
    Compute predicted measurement for each sigma point
    */
    
    for (int i=0;i<2*(2*STATE_DIMENSIONS+1);i++) {
        measurement_sigma[i] = drivetrain.get_measurement(points_sigma[i]);
    }
}

Measurement& Measurement::operator+=(const Measurement& rhs) {
    this->l += rhs.l;
    this->r += rhs.r;
    this->f += rhs.f;
    this->b += rhs.b;
    this->rotation += rhs.rotation;
    this->heading += rhs.heading;
}

inline Measurement operator+(const Measurement& lhs, const Measurement& rhs) {
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
}

inline Measurement operator-(const Measurement& lhs, const Measurement& rhs) {
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
}

Measurement& Measurement::operator*(const Measurement& rhs) {
    this->l *= rhs.l;
    this->r *= rhs.r;
    this->f *= rhs.f;
    this->b *= rhs.b;
    this->rotation *= rhs.rotation;
    this->heading *= rhs.heading;
}

void UKF::combine_predicted_measurements() {
    /*
    */

    mean_weight[0]=1-num_states/(pow(alpha,2)*(num_states+kappa));

    for (int i=1;i<2*num_states;i++) {
        mean_weight[i] = 1/(2*pow(alpha,2)*(num_states+kappa));
    }

    // Reset predicted_measurement to zero first
    predicted_measurement = {0,0,0,0,0,0};

    for (int i=0; i<2*num_states; i++) {
        predicted_measurement += measurement_sigma[i] * mean_weight[i];
    }
}

void UKF::est_cov_predicted_measurements() {
    /*
    INCOMPLETE - NEED TO ADD NOISE MATRIX
    */

    cov_weight[0] = (2 - pow(alpha,2) + beta) - num_states/(pow(alpha,2)*(num_states + kappa));

    for (int i=0;i<2*num_states;i++) {
        if (i>0) {
            cov_weight[i] = 1/(2*pow(alpha,2)*(num_states + kappa));
        }

        measurement_vals[0][i] = measurement_sigma[i].l;
        measurement_vals[1][i] = measurement_sigma[i].r;
        measurement_vals[2][i] = measurement_sigma[i].f;
        measurement_vals[3][i] = measurement_sigma[i].b;
        measurement_vals[4][i] = measurement_sigma[i].rotation;
        measurement_vals[5][i] = measurement_sigma[i].heading;
    }

    measurement_vals[0][2*num_states+1] = predicted_measurement.l;
    measurement_vals[1][2*num_states+1] = predicted_measurement.r;
    measurement_vals[2][2*num_states+1] = predicted_measurement.f;
    measurement_vals[3][2*num_states+1] = predicted_measurement.b;
    measurement_vals[4][2*num_states+1] = predicted_measurement.rotation;
    measurement_vals[5][2*num_states+1] = predicted_measurement.heading;

    for (int i=0;i<MEASUREMENT_DIMENSIONS;i++) {
        for (int j=0;j<MEASUREMENT_DIMENSIONS;j++) {
            measurement_cov[i][j] = 0;
            for (int k=0;k<2*num_states;k++) {
                measurement_cov[i][j] += cov_weight[k]*(measurement_vals[i][k]-measurement_vals[i][2*num_states+1]) * (measurement_vals[j][k]-measurement_vals[j][2*num_states+1]);
            }
            // Add measurement noise R to the measurement covariance
            measurement_cov[i][j] += measurement_noise[i][j];
        }
    }
}

void UKF::state_to_list() {
    for (int i=0;i<2*num_states;i++) {
        if (i>0) {
            cov_weight[i] = 1/(2*pow(alpha,2)*(num_states + kappa));
        }

        state_vals[0][i] = points_sigma[i].x;
        state_vals[1][i] = points_sigma[i].y;
        state_vals[2][i] = points_sigma[i].heading;
        state_vals[3][i] = points_sigma[i].lv;
        state_vals[4][i] = points_sigma[i].av;
        state_vals[5][i] = points_sigma[i].imu_bias;
        state_vals[6][i] = points_sigma[i].encoder_bias;
    }

    state_vals[0][2*num_states+1] = predicted_state.x;
    state_vals[1][2*num_states+1] = predicted_state.y;
    state_vals[2][2*num_states+1] = predicted_state.heading;
    state_vals[3][2*num_states+1] = predicted_state.lv;
    state_vals[4][2*num_states+1] = predicted_state.av;
    state_vals[5][2*num_states+1] = predicted_state.imu_bias;
    state_vals[6][2*num_states+1] = predicted_state.encoder_bias;
}

void UKF::est_cross_cov() {
    /*
    State * Measurement function to return a matrix xy
    Cross-covariance between state and measurement
    */
    
    // Populate state and measurement arrays
    state_to_list();
    
    // Initialize cross-covariance matrix
    for (int i=0;i<STATE_DIMENSIONS;i++) {
        for (int j=0;j<MEASUREMENT_DIMENSIONS;j++) {
            se_me_cov[i][j] = 0;
            // Include all sigma points (start from k=0, not k=1)
            for (int k=0;k<2*num_states;k++) {
                se_me_cov[i][j] += cov_weight[k] * 
                    (state_vals[i][k]-state_vals[i][2*num_states+1]) * 
                    (measurement_vals[j][k]-measurement_vals[j][2*num_states+1]);
            }
        }
    }
}

void UKF::obtain_est() {
    /*
    Obtain the estimated state and state estimation error covariance at time step k.
    */
    
    // Get actual measurement from drivetrain
    Measurement actual_measurement = drivetrain.get_measurement(current);
    
    // Calculate measurement residual
    Measurement residual;
    residual.l = actual_measurement.l - predicted_measurement.l;
    residual.r = actual_measurement.r - predicted_measurement.r;
    residual.f = actual_measurement.f - predicted_measurement.f;
    residual.b = actual_measurement.b - predicted_measurement.b;
    residual.rotation = actual_measurement.rotation - predicted_measurement.rotation;
    residual.heading = actual_measurement.heading - predicted_measurement.heading;
    
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
}

void UKF::predict_set_sigma_points() {
    // Set sigma points using root cP, c = alpha^2 (M + kappa); M -> number of states/sigma points
    points_sigma[0] = next; 

    for (int i=0;i<STATE_DIMENSIONS;i++) {
        std::array<float, STATE_DIMENSIONS> col = UKF::matrix_sqrt(multiply_const_mat(se_cov, scaling_factor))[i];
        offset_sigma[i] = list_to_state(col);

    }

    for (int i=0;i<STATE_DIMENSIONS;i++) {
        std::array<float, STATE_DIMENSIONS> col = UKF::matrix_sqrt(multiply_const_mat(se_cov, -scaling_factor))[i];
        offset_sigma[i+STATE_DIMENSIONS] = list_to_state(col);
    }

    for (int i=0;i<2*num_states;i++) {
        points_sigma[i] = next + offset_sigma[i];
    }
} 

State UKF::state_transition(const State& sigma_point, double left_voltage, double right_voltage, double dt) {
    State next_state;
    
    // Convert voltages to wheel velocities (simplified model)
    // Assume linear relationship between voltage and wheel speed
    const double voltage_to_velocity = 0.1; // This should be calibrated based on your robot
    const double wheel_radius = 0.05; // meters, adjust based on your robot
    const double track_width = 0.3; // meters, distance between wheels, adjust based on your robot
    
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
    next_state.heading = sigma_point.heading + angular_velocity * dt;
    
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
    for (int i = 0; i < 2*num_states; i++) {
        predicted_points_sigma[i] = state_transition(points_sigma[i], left_voltage, right_voltage, dt);
    }
}

void UKF::compute_predicted_states() {
    next = {0,0,0,0,0,0,0};
    for (int i=0;i<2*num_states;i++) {
        next += predicted_points_sigma[i]*mean_weight[i];
    }
}
void UKF::compute_predicted_cov() {
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            predicted_se_cov[i][j] = 0;
        }
    }
    for (int i = 0; i < 2*num_states; i++) {
        State diff;
        diff.x = predicted_points_sigma[i].x - next.x;
        diff.y = predicted_points_sigma[i].y - next.y;
        diff.heading = predicted_points_sigma[i].heading - next.heading;
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
    // choose sigma points
    predict_set_sigma_points();
        // determine each sigma point offset vector
        // determine 

    // apply state transition to sigma points
    apply_state_transition(left_voltage, right_voltage, dt);

    // using state transition compute predicted State
    compute_predicted_states();
        // compute predicted state
        // compute weight_mean for each sigma point
    
    // compute covariance of predicted state
    compute_predicted_cov();
        // compute state estimate error matrix
        // weight covariance for each sigma point from 1-2*num_states -> (2-pow(alpha,2)+beta) - num_states/(pow(alpha, 2) * (num_states + kappa))
} 

void UKF::run(double left_voltage, double right_voltage, double dt) {
    this->correct();
    this->predict(left_voltage, right_voltage, dt);
    time_step++;
}