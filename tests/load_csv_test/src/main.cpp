#include "mems_complementary_filter_interface.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>
#include <algorithm>
#include <cstdint>
#include <iomanip>

#include <Eigen/Geometry>

inline void quaternion_2_euler(const quaternion_t* const _q, Eigen::Vector3f& _e) {
    const double sinp = 2. * (_q->w * _q->y - _q->z * _q->x);
    // Check that pitch is not at a singularity
    if (fabs(sinp) >= 1.) {
        _e[2] = 0.;

        const double R01 = 2. * (_q->x * _q->y - _q->w * _q->z);
        const double R02 = 2. * (_q->x * _q->z + _q->w * _q->y);
        if (sinp >= 0) {    // Gimbal locked down
            const double delta = atan2(R01, R02);
            _e[1] = M_PI_2;
            _e[0] = delta;
        } else {    // Gimbal locked up
            const double delta = atan2(-R01, -R02);
            _e[1] = -M_PI_2;
            _e[0] = delta;
        }
    } else {
        // Pitch, y-axis rotation.
        _e[1] = asin(sinp);

        // Roll, x-axis rotation.
        const double sinr_cosp = 2. * (_q->w * _q->x + _q->y * _q->z);
        const double cosr_cosp = 1. - 2. * (_q->x * _q->x + _q->y * _q->y);
        _e[0] = atan2(sinr_cosp / cos(_e[1]), cosr_cosp / cos(_e[1]));

        // Yaw, z-axis rotation.
        const double siny_cosp = 2. * (_q->w * _q->z + _q->x * _q->y);
        const double cosy_cosp = 1. - 2. * (_q->y * _q->y + _q->z * _q->z);
        // std::cout << "siny_cosp: " << siny_cosp << " cosy_cosp: " << cosy_cosp << std::endl;
        _e[2] = atan2(siny_cosp / cos(_e[1]), cosy_cosp / cos(_e[1]));
    }
}

// Define sensor data structure type alias
using SensorData = std::tuple<
    int64_t,    // timestamp (nanoseconds)
    double,      // wx
    double,      // wy
    double,      // wz
    double,      // ax
    double,      // ay
    double,      // az
    double,      // qw
    double,      // qx
    double,      // qy
    double,      // qz
    double,      // roll_deg
    double,      // pitch_deg
    double       // yaw_deg
>;

using FilterResult = std::tuple<
    int64_t,    // timestamp
    float,      // qw
    float,      // qx
    float,      // qy
    float,      // qz
    float,      // roll_deg
    float,      // pitch_deg
    float       // yaw_deg
>;

// Comparison function for sorting by timestamp
bool compareByTimestamp(const SensorData& a, const SensorData& b) {
    return std::get<0>(a) < std::get<0>(b);
}

// Read sensor data from CSV file
std::vector<SensorData> readSensorDataFromCSV(const std::string& filename, bool sortByTimestamp = true) {
    std::vector<SensorData> sensorData;
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return sensorData;  // Return empty vector
    }
    
    std::string line;
    
    // Skip header line
    std::getline(file, line);
    
    int lineNum = 1;
    int skippedLines = 0;
    
    while (std::getline(file, line)) {
        lineNum++;
        
        // Skip empty lines
        if (line.empty()) {
            skippedLines++;
            continue;
        }
        
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        // Split line by comma
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        // Check if there are enough columns
        if (tokens.size() != 14) {
            std::cerr << "Warning: Line " << lineNum << " has " << tokens.size() 
                      << " columns (expected 14), skipping." << std::endl;
            skippedLines++;
            continue;
        }
        
        try {
            // Parse each column of data
            int64_t timestamp = std::stoull(tokens[0]);
            double wx = std::stod(tokens[1]);
            double wy = std::stod(tokens[2]);
            double wz = std::stod(tokens[3]);
            double ax = std::stod(tokens[4]);
            double ay = std::stod(tokens[5]);
            double az = std::stod(tokens[6]);
            double qw = std::stod(tokens[7]);
            double qx = std::stod(tokens[8]);
            double qy = std::stod(tokens[9]);
            double qz = std::stod(tokens[10]);
            double roll_deg = std::stod(tokens[11]);
            double pitch_deg = std::stod(tokens[12]);
            double yaw_deg = std::stod(tokens[13]);
            
            // Create tuple and add to vector
            sensorData.emplace_back(
                timestamp, wx, wy, wz, ax, ay, az,
                qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg
            );
            
        } catch (const std::exception& e) {
            std::cerr << "Error parsing line " << lineNum << ": " << e.what() 
                      << ", skipping." << std::endl;
            skippedLines++;
            continue;
        }
    }
    
    file.close();
    
    // Sort by timestamp
    if (sortByTimestamp && !sensorData.empty()) {
        std::sort(sensorData.begin(), sensorData.end(), compareByTimestamp);
    }
    
    // Output reading statistics
    std::cout << "Successfully read " << sensorData.size() << " data points from " 
              << filename << std::endl;
    if (skippedLines > 0) {
        std::cout << "Skipped " << skippedLines << " invalid lines." << std::endl;
    }
    
    return sensorData;
}

// Helper function: print a single sensor data point
void printSensorData(const SensorData& data) {
    std::cout << "Timestamp: " << std::get<0>(data) << std::endl;
    std::cout << "Gyro: (" << std::get<1>(data) << ", " 
              << std::get<2>(data) << ", " << std::get<3>(data) << ")" << std::endl;
    std::cout << "Accel: (" << std::get<4>(data) << ", " 
              << std::get<5>(data) << ", " << std::get<6>(data) << ")" << std::endl;
    std::cout << "Quaternion: (" << std::get<7>(data) << ", " 
              << std::get<8>(data) << ", " << std::get<9>(data) << ", " 
              << std::get<10>(data) << ")" << std::endl;
    std::cout << "Euler Angles (deg): (" << std::get<11>(data) << ", " 
              << std::get<12>(data) << ", " << std::get<13>(data) << ")" << std::endl;
    std::cout << std::endl;
}

// Write filter results to CSV file
void writeFilterResultToCSV(const std::string& filename, const std::vector<FilterResult>& results) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }
    
    // Set floating point output precision
    file << std::fixed << std::setprecision(8);
    
    // Write CSV header row
    file << "timestamp,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg\n";
    
    // Write data rows
    for (const auto& result : results) {
        const auto& [timestamp, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg] = result;
        
        file << timestamp << ","
             << qw << ","
             << qx << ","
             << qy << ","
             << qz << ","
             << roll_deg << ","
             << pitch_deg << ","
             << yaw_deg << "\n";
    }
    
    file.close();
    
    std::cout << "Filter results written to " << filename << " (" 
              << results.size() << " rows)" << std::endl;
}

// Example usage
int main() {
    std::string filename = "../data/TDK_sample.csv";
    
    // Read data
    std::vector<SensorData> data = readSensorDataFromCSV(filename, false);
    
    if (data.empty()) {
        std::cout << "No data loaded. Exiting." << std::endl;
        return 1;
    }
    
    // Output statistics
    std::cout << "\nDataset Statistics:" << std::endl;
    std::cout << "Total data points: " << data.size() << std::endl;
    
    // Check if timestamps are ordered
    bool isOrdered = true;
    for (size_t i = 1; i < data.size(); ++i) {
        if (std::get<0>(data[i]) < std::get<0>(data[i-1])) {
            isOrdered = false;
            break;
        }
    }
    std::cout << "Data is " << (isOrdered ? "ordered" : "not ordered") 
              << " by timestamp." << std::endl;
    
    // Output first and last timestamp
    auto firstTimestamp = std::get<0>(data.front());
    auto lastTimestamp = std::get<0>(data.back());
    std::cout << "First timestamp: " << firstTimestamp << " (" 
              << firstTimestamp / 1e9 << " seconds)" << std::endl;
    std::cout << "Last timestamp: " << lastTimestamp << " (" 
              << lastTimestamp / 1e9 << " seconds)" << std::endl;
    std::cout << "Duration: " << (lastTimestamp - firstTimestamp) / 1e9 
              << " seconds" << std::endl;
    
    // Print first few data points as example
    std::cout << "\nFirst 3 data points:" << std::endl;
    for (int i = 0; i < std::min(3, static_cast<int>(data.size())); ++i) {
        std::cout << "Data point " << i + 1 << ":" << std::endl;
        printSensorData(data[i]);
    }
    
    // Calculate time interval statistics
    if (data.size() >= 2) {
        std::vector<double> intervals;
        intervals.reserve(data.size() - 1);
        
        for (size_t i = 1; i < data.size(); ++i) {
            uint64_t intervalNs = std::get<0>(data[i]) - std::get<0>(data[i-1]);
            intervals.push_back(intervalNs / 1e6);  // Convert to milliseconds
        }
        
        // Calculate mean value
        double sum = 0.0;
        for (double interval : intervals) {
            sum += interval;
        }
        double meanInterval = sum / intervals.size();
        
        // Calculate minimum and maximum values
        double minInterval = *std::min_element(intervals.begin(), intervals.end());
        double maxInterval = *std::max_element(intervals.begin(), intervals.end());
        
        std::cout << "\nTime Interval Statistics:" << std::endl;
        std::cout << "Mean interval: " << meanInterval << " ms" << std::endl;
        std::cout << "Min interval: " << minInterval << " ms" << std::endl;
        std::cout << "Max interval: " << maxInterval << " ms" << std::endl;
        std::cout << "Data frequency: " << 1000.0 / meanInterval << " Hz" << std::endl;
    }
    
    const parameters_t parameters {
        .gravity = 9.81f,
        .angular_velocity_threshold = 0.008f,
        .acceleration_threshold = 0.15f,
        .delta_angular_velocity_threshold = 0.005f,
        .gain_acc = 0.0005f,
        .gain_mag = 0.002f,
        .bias_alpha = 0.002f,
        // .default_gyro_bias = {0.00236651, -0.00196377, -4.22027e-05},
        // .default_acc_bias = {0.0549468, 0.511661, 0.014574},
        .default_gyro_bias = {0.f, 0.f, 0.f},
        .default_acc_bias = {0.f, 0.f, -0.f},
        .do_bias_estimation = true,
        .do_adaptive_gain = true,
        .use_magnetometer = false,
        .output_relative_yaw = false
    };

    mems_complementary_filter_t* p_filter;
    filter_create(&parameters, &p_filter);

    std::vector<FilterResult> result;
    int64_t last_ts = 0;
    for (const auto& [timestamp, wx, wy, wz, ax, ay, az, 
                    qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg] : data) {
        const imu_sample_t measurements {
            .timestamp = timestamp,
            .ax = static_cast<float>(ax),
            .ay = static_cast<float>(ay),
            .az = static_cast<float>(az),
            .wx = static_cast<float>(wx),
            .wy = static_cast<float>(wy),
            .wz = static_cast<float>(wz)
        };

        // std::chrono::steady_clock::time_point ts = std::chrono::steady_clock::now();
        filter_add_imu_measurement(p_filter, &measurements);
        // std::chrono::steady_clock::time_point te = std::chrono::steady_clock::now();
        // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(te - ts);
        // std::cout << "update time used: " << time_used.count() << " s." << std::endl;

        quaternion_t quat;
        Eigen::Vector3f euler(0.f, 0.f, 0.f);
        if (filter_get_orientation(p_filter, &quat)) {
            quaternion_2_euler(&quat, euler);
            // std::cout << "get e: " << euler[0] << " " << euler[1] << " " << euler[2] << std::endl;
            euler *= 57.3f;  // Convert radians to degrees
            result.emplace_back(std::forward_as_tuple(quat.timestamp,
                quat.w, quat.x, quat.y, quat.z, euler[0], euler[1], euler[2]));
        }

        last_ts = timestamp;
    }
    
    // Destroy filter
    filter_destroy(p_filter);
    
    // Write filter results to CSV file
    std::string output_filename = "../data/filter_result.csv";
    writeFilterResultToCSV(output_filename, result);
    
    // Optional: output statistics of filter results
    std::cout << "\nFilter Result Statistics:" << std::endl;
    std::cout << "Total filter results: " << result.size() << std::endl;
    
    if (!result.empty()) {
        auto first_result_timestamp = std::get<0>(result.front());
        auto last_result_timestamp = std::get<0>(result.back());
        std::cout << "First result timestamp: " << first_result_timestamp << std::endl;
        std::cout << "Last result timestamp: " << last_result_timestamp << std::endl;
        
        // Output first few filter results as example
        std::cout << "\nFirst 3 filter results:" << std::endl;
        for (int i = 0; i < std::min(3, static_cast<int>(result.size())); ++i) {
            const auto& [timestamp, qw, qx, qy, qz, roll_deg, pitch_deg, yaw_deg] = result[i];
            std::cout << "Result " << i + 1 << ":" << std::endl;
            std::cout << "  Timestamp: " << timestamp << std::endl;
            std::cout << "  Quaternion: (" << qw << ", " << qx << ", " << qy << ", " << qz << ")" << std::endl;
            std::cout << "  Euler Angles (deg): (" << roll_deg << ", " << pitch_deg << ", " << yaw_deg << ")" << std::endl;
        }
    }
    
    return 0;
}