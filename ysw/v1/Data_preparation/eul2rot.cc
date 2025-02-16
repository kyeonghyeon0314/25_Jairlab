#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

std::vector<std::vector<double>> euler_to_rotation_matrix(double roll, double pitch, double yaw) {
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    return {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp,     cp * sr,               cp * cr}
    };
}

int main() {
    double roll = 0.1;
    double pitch = 0.2;
    double yaw = 0.3;

    auto rot_matrix = euler_to_rotation_matrix(roll, pitch, yaw);

    // Print the rotation matrix
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "Rotation Matrix:\n";
    
    for (const auto& row : rot_matrix) {
        for (double val : row) {
            std::cout << std::setw(12) << val << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
