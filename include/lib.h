#pragma once
#include "api.h"

struct Point {
    float x;
    float y;
};

struct Pose {
    float x;
    float y;
    double heading;
};

struct State {
    float x;
    float y;
    float heading;
    float lv; // Linear Velocity
    float av; // Angular Velocity
    float imu_bias;
    float encoder_bias;
    State& operator+=(const State& rhs);
    State& operator*(const float factor);
};

struct Measurement {
    float l;
    float r;
    float f;
    float b;
    float rotation;
    float heading;
    Measurement& operator+=(const Measurement& rhs);
    Measurement& operator-=(const Measurement& rhs);
    Measurement& operator*(const float& rhs);
    Measurement& operator*(const Measurement& rhs);
};

inline Measurement operator+(const Measurement& lhs, const Measurement& rhs);
inline Measurement operator-(const Measurement& lhs, const Measurement& rhs);

inline State operator+(const State& lhs, const State& rhs);

const int STATE_DIMENSIONS = 7;
const int MEASUREMENT_DIMENSIONS = 6;

class PID {
    public:
        float kP, kI, kD, settle_error, settle_time, timeout, prev_error, integral, integral_threshold = 0;
        float time_settled, time_spent = 0;
        float dt = 10; // milliseconds
        PID(float kP, float kI, float kD, float settle_error, float settle_time, float timeout);

        float compute(float error);
        bool get_settled(float error);
        void reset();
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
/*
https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
*/
class PurePursuit {
    private:
        float lookahead_distance = 0;
        float max_angular_velocity = 0;
        float desired_linear_velocity = 0;
        float settle_radius = 1;
        std::vector<Point> waypoints;
        int waypoint_index = 1;
        Point goal_point = {0,0};

    public:
        PurePursuit(std::vector<Point> path, float lookahead_radius, float max_angular_velocity = 0, float desired_linear_velocity = 0);
        Point set_goal_point(Point current_pos);
        Point get_goal_point();
        std::vector<Point> get_intersection(Point current_pos, Point pt1, Point pt2, double lookahead_distance); // line-circle intersection, returns list of intersections
        std::vector<Point> get_line();
        std::vector<float> compute_errors(Pose current_pose);
        bool get_settled(Point current_pos);
};

// sources: MATLAB, "Kalman and Bayesian Filters in Python" by Roger R Labbe Jr
class UKF {
    private:
        // paramters
        float alpha, beta, kappa = 0;
        float scaling_factor = 1;

        // Kalman Gain
        float K = 0;

        // Mean, SD of each
        State mean;
        State sd;

        // States
        State current;
        State prev;
        State next;

        // Measured State
        State measured;

        // each time step: state, measurement, 
        int num_states = 2*STATE_DIMENSIONS+1;// 2n+1; n is number of dimensions of the state

        int time_step = 0; // number of cycles, every 10 ms is 1 cycle?

        // matrices
        std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> process_noise;
        std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> measurement_noise;
        std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> se_cov; // state estimation error covariance matrix
        std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> measurement_cov; // measurement covariance matrix
        std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> se_me_cov; // cross covariance between state prediction and measurement


        // sigma vector
        State points_sigma[2*(2*STATE_DIMENSIONS+1)];
        State offset_sigma[2*(2*STATE_DIMENSIONS+1)]; // Predicted sigma estimates
        Measurement measurement_sigma[2*(2*STATE_DIMENSIONS+1)];
        Measurement predicted_measurement;
        std::array<float, 2*(2*STATE_DIMENSIONS+1)> mean_weight;
        std::array<float, 2*STATE_DIMENSIONS+1> cov_weight;
        

        // Reference to drivetrain for measurements
        OWDrivetrain& drivetrain;

        // possible util functions: covariance matrix functions
        std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix_sqrt(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix);
        std::array<float, STATE_DIMENSIONS> col_sqrt(std::array<float, STATE_DIMENSIONS> col);
        State list_to_state(std::array<float, STATE_DIMENSIONS> col);
    public:
        UKF(OWDrivetrain& drivetrain, State state, State mean, State sd, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> p_noise, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> m_noise, float alpha, float beta, float kappa, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> se_cov); // initialize state
        State get_estimate();
        void set_num_sigma_points(); // Set number of sigma points
        void set_sigma_points(); // Set sigma points using root cP, c = alpha^2 (M + kappa); M -> number of states/sigma points
        void compute_predicted_measurements();
        void combine_predicted_measurements();
        void est_cov_predicted_measurements();
        void est_cross_cov();
        void kalman_gain();

        // Macro Functions
        void update(); // update state & state estimation error covariance. collect data from drivetrain. compute
        void predict(); // 
        void correct();
        void run();
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
        float motor_speed = 600;
        float wheel_diameter = 2.75;

        float left_tracker_offset;
        float right_tracker_offset;
        float back_tracker_offset;
        float total_l_dist, total_r_dist, total_s_dist, left_pos, right_pos, back_pos, prev_left_pos, prev_right_pos, prev_b_pos, delta_l_dist, delta_r_dist, delta_s_dist, reset_l_pos, reset_r_pos, reset_s_pos = 0;
        float prev_drive_x, prev_drive_y, drive_x, drive_y, x_offset, y_offset, x_global_offset, y_global_offset = 0;
        float avg_orientation, prev_orientation, orientation, reset_orientation, delta_theta = 0;

        Point target_point = {0,0};
        std::vector<Point> path = {{0,0}, {0,0}};

        pros::Task *odom_task = nullptr;
        pros::Task *move_task = nullptr;
        PID linear_pid;
        PID turn_pid;

        int tracking_type = 1;
    public:
        Drivetrain(std::vector<std::int8_t> left_motor_ports, std::vector<std::int8_t> right_motor_ports, PID linear_pid, PID turn_pid, int motor_speed, int imu_port, int left_tracker_port, int right_tracker_port, int back_tracker_port, float track_width, float left_tracker_offset, float right_tracker_offset, float back_tracker_offset);
        Point get_position();
        Pose get_pose();
        void update_position();
        void reset_position();
        void move(double left_voltage, double right_voltage);
        void move_to_point(Point target_point);
        static void move_wrapper(void *param);
        void start_odom();
        static void odom_wrapper(void *param);
};

class OWDrivetrain { // one wheel tracker drivetrain
    private:
        pros::MotorGroup left_motors;
        pros::MotorGroup right_motors;
        pros::Rotation left_tracker;

        float track_width;
        float motor_speed = 600;
        float wheel_diameter = 2.75;

        float left_tracker_offset;
        float total_l_dist, total_r_dist, left_pos, prev_left_pos, delta_l_dist, reset_l_pos = 0;
        float prev_drive_x, prev_drive_y, drive_x, drive_y, x_offset, y_offset, x_global_offset, y_global_offset = 0;
        float heading = 0;
        float turn_error = 0;
        float turn_target = 0;

        Point target_point = {0,0};
        std::vector<Point> path = {{0,0}, {0,0}};

        pros::Task *odom_task = nullptr;
        pros::Task *move_task = nullptr;
        pros::Task *turn_task = nullptr;
        PID linear_pid;
        PID turn_pid;

        int tracking_type = 1;

        // DISTANCE CONFIG
        pros::Distance left_dist;
        pros::Distance right_dist;
        pros::Distance front_dist;
        pros::Distance back_dist;

        // MONTE CARLO LOCALIZATION
        int mcl_particle_limit;
        bool mcl_global_localization;
        Pose mcl_pose_initial;
        double mcl_covariance_initial;

        State current_state;

        // UKF for state estimation
        UKF* ukf;

    public:
        OWDrivetrain(std::vector<std::int8_t> left_motor_ports, std::vector<std::int8_t> right_motor_ports, PID linear_pid, PID turn_pid, int motor_speed, int imu_port, int imu2_port, int left_tracker_port, float track_width, float left_tracker_offset, int left_dist_port, int right_dist_port, int front_dist_port, int back_dist_port);
        Point get_position();
        State get_state(); // from input measurement apply state transition model.
        Measurement get_measurement(State input);
        Pose get_pose();
        void update_position();
        void reset_position();
        void move(double left_voltage, double right_voltage);
        void move_to_point(Point target_point);
        static void move_wrapper(void *param);
        void start_odom();
        static void odom_wrapper(void *param);
        void turn_to(double heading);
        static void turn_wrapper(void *param);

        void run_display();
        void test_lidar_display(); // Debug function to display LiDAR position on brain

        Point dist_est_pos();
        Point tracker_est_pos();

        pros::Imu imu;
        pros::Imu imu2;
};