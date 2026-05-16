# shao-lib

A VEX V5 robotics library built on [PROS](https://pros.cs.purdue.edu/), providing autonomous motion control, odometry, and localization for competition robots.

## Features

- **PID Controller** — Configurable PID with settling, timeout, and integral windup protection
- **Pure Pursuit** — Path-following controller that drives a robot along a series of waypoints
- **Odometry** — Position tracking using rotation sensors and IMU
- **Unscented Kalman Filter (UKF)** — State estimation fusing encoder and IMU data
- **Distance Sensor Localization** — Wall-relative position estimation using 4 distance sensors
- **Drivetrain Classes** — Ready-to-use drivetrain abstractions with async odometry and motion tasks

## Classes

### `PID`

A standard PID controller with settling detection.

```cpp
PID linear(
    0.5,   // kP
    0.01,  // kI
    0.1,   // kD
    0.1,   // settle_error (inches)
    200,   // settle_time (ms)
    2000   // timeout (ms)
);

float output = linear.compute(error);
bool done = linear.get_settled(error);
linear.reset();
```

### `PurePursuit`

Follows a series of waypoints using the pure pursuit algorithm. Computes lateral and angular errors for the drivetrain to act on.

```cpp
std::vector<Point> path = {{0,0}, {24,0}, {24,24}};
PurePursuit pp(path, 6.0 /*lookahead*/, 200 /*max_angular_vel*/, 100 /*linear_vel*/);

Point goal = pp.set_goal_point(current_pos);
std::vector<float> errors = pp.compute_errors(current_pose); // [linear, angular]
bool done = pp.get_settled(current_pos);
```

### `Drivetrain`

Full three-tracker drivetrain (left, right, back rotation sensors + IMU) with odometry and point-to-point movement.

```cpp
Drivetrain drivetrain(
    {1, 2},        // left motor ports
    {-3, -4},      // right motor ports
    linear_pid, turn_pid,
    600,           // motor RPM
    10,            // IMU port
    11, 12, 13,    // left, right, back tracker ports
    11.5,          // track width (inches)
    0.0, 0.0, 2.0  // tracker offsets (inches)
);

drivetrain.start_odom();
drivetrain.move_to_point({24, 0});
Point pos = drivetrain.get_position();
Pose pose = drivetrain.get_pose();
```

### `OWDrivetrain`

Single-tracker drivetrain using one rotation sensor + two IMUs + four distance sensors. Supports UKF state estimation and distance-sensor-based localization.

```cpp
OWDrivetrain drivetrain(
    {1, 2}, {-3, -4},       // left/right motor ports
    linear_pid, turn_pid,
    600,                    // motor RPM
    13, 14,                 // IMU ports
    15,                     // tracker port
    11.5, 6.7,              // track width, tracker offset
    fx, fy,                 // front distance sensor offsets (inches)
    bx, by,                 // back distance sensor offsets
    lx, ly,                 // left distance sensor offsets
    rx, ry                  // right distance sensor offsets
);

drivetrain.start_odom();
drivetrain.move_to_point({24, 0});
drivetrain.turn_to(90.0);   // degrees
Point pos = drivetrain.get_position();
```

### `UKF`

Unscented Kalman Filter for fusing encoder and IMU measurements into a smooth state estimate. Operates on a 7-dimensional state vector `[x, y, heading, linear_vel, angular_vel, imu_bias, encoder_bias]`.

```cpp
UKF ukf(drivetrain, initial_state, mean, sd,
        process_noise, measurement_noise,
        alpha, beta, kappa, se_cov);

ukf.run();   // predict + correct cycle
State est = ukf.get_estimate();
```

## Utility Functions

Declared in `util.h`:

| Function | Description |
|---|---|
| `distance(p1, p2)` | Euclidean distance between two `Point`s |
| `min_angle(angle)` | Normalize angle to `[-180, 180]` |
| `deg_to_rad(angle)` | Degrees to radians |
| `rad_to_deg(angle)` | Radians to degrees |
| `sgn(num)` | Sign of a number |
| `average(list)` | Average of a `vector<double>` |
| `POI(line1, line2)` | Point of intersection of two lines |
| Matrix utilities | `add_mat`, `multiply_const_mat`, `det_mat_77`, `trace_mat_77` |

## Global Configuration

Hardware is declared in `global.h` / `global.cpp`:

```cpp
// Distance sensors (4 sides)
pros::Distance left_dist, right_dist, front_dist, back_dist;

// Dual IMUs
pros::Imu imu, imu2;

// Subsystems
pros::Motor intake, stick;
pros::adi::DigitalOut loader;

// Motor port lists
std::vector<std::int8_t> left_mg, right_mg;
```

## Coordinate System

- Origin `(0, 0)` is field center
- **+X** = right, **-X** = left
- **+Y** = forward, **-Y** = backward
- Heading `0` = north (forward), increases clockwise
- Field is 144 x 144 inches (12 x 12 feet)

## Building

This project uses the standard PROS build system:

```bash
pros build
pros upload
```

## Dependencies

- [PROS](https://pros.cs.purdue.edu/) kernel for VEX V5
- VEX V5 Brain, motors, rotation sensors, IMU, and distance sensors
