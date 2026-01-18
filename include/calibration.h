#include <vector>
#include <cmath>
#include <algorithm>
#include "api.h"

// ==================== UTILITY FUNCTIONS ====================

struct SensorStats {
    float mean;
    float variance;
    float std_dev;
    float min_val;
    float max_val;
    int sample_count;
};

// Calculate statistics from a vector of readings
SensorStats calculateStats(const std::vector<float>& readings) {
    SensorStats stats;
    stats.sample_count = readings.size();
    
    if (readings.empty()) {
        stats.mean = stats.variance = stats.std_dev = 0.0f;
        stats.min_val = stats.max_val = 0.0f;
        return stats;
    }
    
    // Calculate mean
    stats.mean = 0.0f;
    for (float val : readings) {
        stats.mean += val;
    }
    stats.mean /= readings.size();
    
    // Calculate variance
    stats.variance = 0.0f;
    for (float val : readings) {
        float diff = val - stats.mean;
        stats.variance += diff * diff;
    }
    stats.variance /= readings.size();
    
    // Standard deviation
    stats.std_dev = std::sqrt(stats.variance);
    
    // Min and max
    stats.min_val = *std::min_element(readings.begin(), readings.end());
    stats.max_val = *std::max_element(readings.begin(), readings.end());
    
    return stats;
}

// ==================== SENSOR CALIBRATION CLASS ====================

class SensorCalibration {
private:
    pros::Imu& imu;
    pros::Imu& imu2;
    pros::Distance& front_dist;
    pros::Distance& back_dist;
    pros::Distance& left_dist;
    pros::Distance& right_dist;
    pros::Rotation& left_encoder;
    pros::Rotation& right_encoder;
    
public:
    SensorCalibration(
        pros::Imu& imu1, pros::Imu& imu2,
        pros::Distance& front, pros::Distance& back,
        pros::Distance& left, pros::Distance& right,
        pros::Rotation& left_enc, pros::Rotation& right_enc
    ) : imu(imu1), imu2(imu2),
        front_dist(front), back_dist(back),
        left_dist(left), right_dist(right),
        left_encoder(left_enc), right_encoder(right_enc) {}
    
    // ==================== IMU CALIBRATION ====================
    
    // Calibrate stationary IMU noise
    SensorStats calibrateIMUStationary(int samples = 200, int delay_ms = 20) {
        std::vector<float> heading_readings;
        
        pros::lcd::set_text(1, "Calibrating IMU (stationary)...");
        pros::lcd::set_text(2, "DO NOT MOVE ROBOT!");
        
        // Collect samples
        for (int i = 0; i < samples; i++) {
            float heading = imu.get_heading() * M_PI / 180.0f; // Convert to radians
            heading_readings.push_back(heading);
            
            // Progress indicator
            if (i % 20 == 0) {
                pros::lcd::set_text(3, "Progress: " + std::to_string(i) + "/" + std::to_string(samples));
            }
            
            pros::delay(delay_ms);
        }
        
        SensorStats stats = calculateStats(heading_readings);
        
        pros::lcd::set_text(4, "IMU Std Dev: " + std::to_string(stats.std_dev) + " rad");
        pros::lcd::set_text(5, "IMU Variance: " + std::to_string(stats.variance) + " rad^2");
        
        return stats;
    }
    
    // Calibrate IMU drift during rotation
    struct IMUDriftStats {
        SensorStats imu1_stats;
        SensorStats imu2_stats;
        float drift_error;  // Difference between expected and actual rotation
    };
    
    IMUDriftStats calibrateIMURotation(float target_degrees = 360.0f, int samples = 100) {
        IMUDriftStats drift_stats;
        std::vector<float> imu1_readings;
        std::vector<float> imu2_readings;
        
        pros::lcd::set_text(1, "Calibrating IMU (rotation)...");
        pros::lcd::set_text(2, "Robot will rotate " + std::to_string(target_degrees) + " degrees");
        
        // Record initial heading
        float initial_heading1 = imu.get_heading();
        float initial_heading2 = imu2.get_heading();
        
        // TODO: Add your rotation command here
        // rotateRobot(target_degrees);
        
        pros::delay(500); // Wait for rotation to complete
        
        // Collect samples during/after rotation
        for (int i = 0; i < samples; i++) {
            float heading1 = imu.get_heading() * M_PI / 180.0f;
            float heading2 = imu2.get_heading() * M_PI / 180.0f;
            
            imu1_readings.push_back(heading1);
            imu2_readings.push_back(heading2);
            
            pros::delay(20);
        }
        
        drift_stats.imu1_stats = calculateStats(imu1_readings);
        drift_stats.imu2_stats = calculateStats(imu2_readings);
        
        // Calculate drift error
        float final_heading1 = imu.get_heading();
        float actual_rotation = final_heading1 - initial_heading1;
        drift_stats.drift_error = std::abs(actual_rotation - target_degrees);
        
        pros::lcd::set_text(4, "IMU1 Variance: " + std::to_string(drift_stats.imu1_stats.variance));
        pros::lcd::set_text(5, "Drift Error: " + std::to_string(drift_stats.drift_error) + " deg");
        
        return drift_stats;
    }
    
    // ==================== DISTANCE SENSOR CALIBRATION ====================
    
    struct DistanceSensorStats {
        SensorStats front;
        SensorStats back;
        SensorStats left;
        SensorStats right;
    };
    
    DistanceSensorStats calibrateDistanceSensors(float known_distance_mm = 300.0f, 
                                                   int samples = 150, 
                                                   int delay_ms = 20) {
        DistanceSensorStats dist_stats;
        std::vector<float> front_readings, back_readings, left_readings, right_readings;
        
        pros::lcd::set_text(1, "Calibrating Distance Sensors...");
        pros::lcd::set_text(2, "Place robot " + std::to_string(known_distance_mm) + "mm from walls");
        pros::lcd::set_text(3, "Press A when ready...");
        
        // Wait for button press
        while (!pros::lcd::read_buttons() & LCD_BTN_CENTER) {
            pros::delay(20);
        }
        
        pros::lcd::set_text(3, "Collecting data...");
        
        // Collect samples
        for (int i = 0; i < samples; i++) {
            int front_val = front_dist.get();
            int back_val = back_dist.get();
            int left_val = left_dist.get();
            int right_val = right_dist.get();
            
            // Convert to inches and store
            if (front_val != -1) front_readings.push_back(front_val / 25.4f);
            if (back_val != -1) back_readings.push_back(back_val / 25.4f);
            if (left_val != -1) left_readings.push_back(left_val / 25.4f);
            if (right_val != -1) right_readings.push_back(right_val / 25.4f);
            
            if (i % 15 == 0) {
                pros::lcd::set_text(4, "Progress: " + std::to_string(i) + "/" + std::to_string(samples));
            }
            
            pros::delay(delay_ms);
        }
        
        dist_stats.front = calculateStats(front_readings);
        dist_stats.back = calculateStats(back_readings);
        dist_stats.left = calculateStats(left_readings);
        dist_stats.right = calculateStats(right_readings);
        
        // Display results
        pros::lcd::clear();
        pros::lcd::set_text(1, "Front: σ=" + std::to_string(dist_stats.front.std_dev) + 
                              " var=" + std::to_string(dist_stats.front.variance));
        pros::lcd::set_text(2, "Back:  σ=" + std::to_string(dist_stats.back.std_dev) + 
                              " var=" + std::to_string(dist_stats.back.variance));
        pros::lcd::set_text(3, "Left:  σ=" + std::to_string(dist_stats.left.std_dev) + 
                              " var=" + std::to_string(dist_stats.left.variance));
        pros::lcd::set_text(4, "Right: σ=" + std::to_string(dist_stats.right.std_dev) + 
                              " var=" + std::to_string(dist_stats.right.variance));
        
        return dist_stats;
    }
    
    // Test individual distance sensor quality
    void testDistanceSensorQuality(pros::Distance& sensor, const std::string& name) {
        std::vector<float> readings;
        
        pros::lcd::clear();
        pros::lcd::set_text(1, "Testing " + name + " sensor quality");
        pros::lcd::set_text(2, "Move sensor toward/away from wall");
        pros::lcd::set_text(3, "Press A to stop");
        
        while (!(pros::lcd::read_buttons() & LCD_BTN_CENTER)) {
            int val = sensor.get();
            if (val != -1) {
                float inches = val / 25.4f;
                readings.push_back(inches);
                
                // Show live reading
                pros::lcd::set_text(4, "Current: " + std::to_string(inches) + " in");
                pros::lcd::set_text(5, "Samples: " + std::to_string(readings.size()));
            }
            
            pros::delay(50);
        }
        
        SensorStats stats = calculateStats(readings);
        
        pros::lcd::clear();
        pros::lcd::set_text(1, name + " Results:");
        pros::lcd::set_text(2, "Mean: " + std::to_string(stats.mean) + " in");
        pros::lcd::set_text(3, "Std Dev: " + std::to_string(stats.std_dev) + " in");
        pros::lcd::set_text(4, "Variance: " + std::to_string(stats.variance) + " in^2");
        pros::lcd::set_text(5, "Range: [" + std::to_string(stats.min_val) + ", " + 
                              std::to_string(stats.max_val) + "]");
    }
    
    // ==================== ROTATION SENSOR CALIBRATION ====================
    
    struct RotationSensorStats {
        SensorStats left;
        SensorStats right;
        float wheel_slippage;  // Difference between left and right
    };
    
    RotationSensorStats calibrateRotationSensors(float drive_distance_inches = 120.0f,
                                                   int samples = 100,
                                                   int delay_ms = 20) {
        RotationSensorStats rot_stats;
        std::vector<float> left_readings, right_readings;
        
        pros::lcd::set_text(1, "Calibrating Rotation Sensors...");
        pros::lcd::set_text(2, "Robot will drive " + std::to_string(drive_distance_inches) + " inches");
        
        // Reset encoders
        left_encoder.reset_position();
        right_encoder.reset_position();
        
        // Record initial positions
        float initial_left = left_encoder.get_position();
        float initial_right = right_encoder.get_position();
        
        // TODO: Add your straight drive command here
        // driveStraight(drive_distance_inches);
        
        pros::delay(500); // Wait for drive to complete
        
        // Collect samples
        for (int i = 0; i < samples; i++) {
            float left_pos = left_encoder.get_position();
            float right_pos = right_encoder.get_position();
            
            left_readings.push_back(left_pos);
            right_readings.push_back(right_pos);
            
            if (i % 10 == 0) {
                pros::lcd::set_text(3, "Progress: " + std::to_string(i) + "/" + std::to_string(samples));
            }
            
            pros::delay(delay_ms);
        }
        
        rot_stats.left = calculateStats(left_readings);
        rot_stats.right = calculateStats(right_readings);
        
        // Calculate slippage (difference between wheels)
        float final_left = left_encoder.get_position();
        float final_right = right_encoder.get_position();
        rot_stats.wheel_slippage = std::abs(final_left - final_right);
        
        pros::lcd::set_text(4, "Left Variance: " + std::to_string(rot_stats.left.variance));
        pros::lcd::set_text(5, "Slippage: " + std::to_string(rot_stats.wheel_slippage) + " ticks");
        
        return rot_stats;
    }
    
    // ==================== COMPREHENSIVE CALIBRATION ====================
    
    struct ComprehensiveCalibration {
        SensorStats imu_stationary;
        IMUDriftStats imu_rotation;
        DistanceSensorStats distance;
        RotationSensorStats rotation;
        
        // Recommended UKF measurement noise matrix (R)
        std::array<std::array<float, 6>, 6> measurement_noise;
    };
    
    ComprehensiveCalibration runFullCalibration() {
        ComprehensiveCalibration cal;
        
        pros::lcd::clear();
        pros::lcd::set_text(1, "===== FULL SENSOR CALIBRATION =====");
        pros::delay(2000);
        
        // Step 1: IMU Stationary
        pros::lcd::clear();
        cal.imu_stationary = calibrateIMUStationary();
        pros::delay(3000);
        
        // Step 2: Distance Sensors
        pros::lcd::clear();
        cal.distance = calibrateDistanceSensors();
        pros::delay(3000);
        
        // Step 3: IMU Rotation (optional)
        // Uncomment if you want to test rotation drift
        // pros::lcd::clear();
        // cal.imu_rotation = calibrateIMURotation();
        // pros::delay(3000);
        
        // Step 4: Rotation Sensors (optional)
        // Uncomment if you want to test encoder drift
        // pros::lcd::clear();
        // cal.rotation = calibrateRotationSensors();
        // pros::delay(3000);
        
        // Generate recommended measurement noise matrix
        cal.measurement_noise = generateMeasurementNoise(cal);
        
        // Display final recommendations
        displayCalibrationResults(cal);
        
        return cal;
    }
    
    // Generate measurement noise matrix from calibration data
    std::array<std::array<float, 6>, 6> generateMeasurementNoise(
        const ComprehensiveCalibration& cal) {
        
        std::array<std::array<float, 6>, 6> R = {};
        
        // Initialize to zero (off-diagonal elements)
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                R[i][j] = 0.0f;
            }
        }
        
        // Distance sensors (use variance directly, or add safety margin)
        R[0][0] = std::max(25.0f, cal.distance.front.variance * 1.5f);  // Front
        R[1][1] = std::max(25.0f, cal.distance.back.variance * 1.5f);   // Back
        R[2][2] = std::max(25.0f, cal.distance.left.variance * 1.5f);   // Left
        R[3][3] = std::max(25.0f, cal.distance.right.variance * 1.5f);  // Right
        
        // IMU heading (use variance with safety margin)
        R[4][4] = std::max(0.0005f, cal.imu_stationary.variance * 2.0f);
        
        // Encoder/tracking (assume ~2 inch std dev if not calibrated)
        R[5][5] = 4.0f;  // Default: 2.0^2
        
        return R;
    }
    
    void displayCalibrationResults(const ComprehensiveCalibration& cal) {
        pros::lcd::clear();
        pros::lcd::set_text(1, "===== CALIBRATION COMPLETE =====");
        pros::lcd::set_text(2, "Recommended R Matrix Values:");
        pros::lcd::set_text(3, "Front: " + std::to_string(cal.measurement_noise[0][0]));
        pros::lcd::set_text(4, "Back:  " + std::to_string(cal.measurement_noise[1][1]));
        pros::lcd::set_text(5, "Left:  " + std::to_string(cal.measurement_noise[2][2]));
        pros::delay(5000);
        
        pros::lcd::clear();
        pros::lcd::set_text(1, "Right: " + std::to_string(cal.measurement_noise[3][3]));
        pros::lcd::set_text(2, "IMU:   " + std::to_string(cal.measurement_noise[4][4]));
        pros::lcd::set_text(3, "Enc:   " + std::to_string(cal.measurement_noise[5][5]));
        pros::lcd::set_text(4, "");
        pros::lcd::set_text(5, "Copy these to your UKF init!");
    }
    
    // ==================== CODE GENERATION ====================
    
    void printMeasurementNoiseCode(const ComprehensiveCalibration& cal) {
        printf("\n===== GENERATED UKF MEASUREMENT NOISE CODE =====\n\n");
        printf("std::array<std::array<float, 6>, 6> measurement_noise = {{\n");
        printf("    {%.4ff, 0.0f,   0.0f,   0.0f,   0.0f,    0.0f},  // Front dist\n", 
               cal.measurement_noise[0][0]);
        printf("    {0.0f,  %.4ff, 0.0f,   0.0f,   0.0f,    0.0f},  // Back dist\n", 
               cal.measurement_noise[1][1]);
        printf("    {0.0f,  0.0f,   %.4ff, 0.0f,   0.0f,    0.0f},  // Left dist\n", 
               cal.measurement_noise[2][2]);
        printf("    {0.0f,  0.0f,   0.0f,   %.4ff, 0.0f,    0.0f},  // Right dist\n", 
               cal.measurement_noise[3][3]);
        printf("    {0.0f,  0.0f,   0.0f,   0.0f,   %.6ff,  0.0f},  // IMU heading\n", 
               cal.measurement_noise[4][4]);
        printf("    {0.0f,  0.0f,   0.0f,   0.0f,   0.0f,    %.4ff}   // Encoder\n", 
               cal.measurement_noise[5][5]);
        printf("}};\n\n");
        
        printf("// Sensor Quality Summary:\n");
        printf("// Front Distance: σ=%.2f in, var=%.2f in²\n", 
               cal.distance.front.std_dev, cal.distance.front.variance);
        printf("// Back Distance:  σ=%.2f in, var=%.2f in²\n", 
               cal.distance.back.std_dev, cal.distance.back.variance);
        printf("// Left Distance:  σ=%.2f in, var=%.2f in²\n", 
               cal.distance.left.std_dev, cal.distance.left.variance);
        printf("// Right Distance: σ=%.2f in, var=%.2f in²\n", 
               cal.distance.right.std_dev, cal.distance.right.variance);
        printf("// IMU Heading:    σ=%.4f rad, var=%.6f rad²\n", 
               cal.imu_stationary.std_dev, cal.imu_stationary.variance);
        printf("\n================================================\n\n");
    }
};

// ==================== USAGE EXAMPLE ====================

void calibrateSensorsForUKF() {
    // Initialize sensors
    pros::Imu imu1(7);
    pros::Imu imu2(8);
    pros::Distance front_dist(11);
    pros::Distance back_dist(12);
    pros::Distance left_dist(13);
    pros::Distance right_dist(14);
    pros::Rotation left_enc(1);
    pros::Rotation right_enc(2);
    
    // Wait for IMU to calibrate
    imu1.reset();
    imu2.reset();
    while (imu1.is_calibrating() || imu2.is_calibrating()) {
        pros::delay(20);
    }
    
    // Create calibration object
    SensorCalibration calibrator(imu1, imu2, 
                                 front_dist, back_dist, left_dist, right_dist,
                                 left_enc, right_enc);
    
    // Run full calibration
    auto results = calibrator.runFullCalibration();
    
    // Print code to terminal
    calibrator.printMeasurementNoiseCode(results);
    
    // The measurement_noise matrix is now in results.measurement_noise
    // Use it to initialize your UKF!
}

// ==================== INTERACTIVE MENU ====================

void sensorCalibrationMenu() {
    pros::Imu imu1(7);
    pros::Imu imu2(8);
    pros::Distance front_dist(11);
    pros::Distance back_dist(12);
    pros::Distance left_dist(13);
    pros::Distance right_dist(14);
    pros::Rotation left_enc(1);
    pros::Rotation right_enc(2);
    
    SensorCalibration calibrator(imu1, imu2, 
                                 front_dist, back_dist, left_dist, right_dist,
                                 left_enc, right_enc);
    
    while (true) {
        pros::lcd::clear();
        pros::lcd::set_text(1, "===== CALIBRATION MENU =====");
        pros::lcd::set_text(2, "LEFT:   Distance Sensors");
        pros::lcd::set_text(3, "CENTER: IMU Stationary");
        pros::lcd::set_text(4, "RIGHT:  Full Calibration");
        
        uint8_t buttons = pros::lcd::read_buttons();
        
        if (buttons & LCD_BTN_LEFT) {
            calibrator.calibrateDistanceSensors();
            pros::delay(5000);
        }
        else if (buttons & LCD_BTN_CENTER) {
            calibrator.calibrateIMUStationary();
            pros::delay(5000);
        }
        else if (buttons & LCD_BTN_RIGHT) {
            auto results = calibrator.runFullCalibration();
            calibrator.printMeasurementNoiseCode(results);
            break;
        }
        
        pros::delay(100);
    }
}