// cpp version of quaternion_matrix() in https://github.com/MichaelGrupp/evo/blob/master/evo/core/transformations.py#L1258

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

const double _EPS = 1e-15; // Small threshold for near-zero quaternion magnitudes

std::vector<std::vector<double>> quaternion_matrix(const std::vector<double>& quat) {
    std::vector<double> q = quat;
    
    // Calculate quaternion magnitude squared
    double n = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    
    // Return identity for near-zero quaternions
    if (n < _EPS) {
        return {
            {1,0,0},
            {0,1,0},
            {0,0,1}
        };
    }
    
    // Normalization factor
    double scale = sqrt(2.0 / n);
    for (auto& val : q) val *= scale;

    // Calculate outer product matrix (not vector product)
    std::vector<std::vector<double>> outer(4, std::vector<double>(4));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            outer[i][j] = q[i] * q[j];
            
    return {
        {1.0 - outer[2][2] - outer[3][3], outer[1][2] - outer[3][0], outer[1][3] + outer[2][0]},
        {outer[1][2] + outer[3][0], 1.0 - outer[1][1] - outer[3][3], outer[2][3] - outer[1][0]},
        {outer[1][3] - outer[2][0], outer[2][3] + outer[1][0], 1.0 - outer[1][1] - outer[2][2]}
    };
}

int main() {
    std::vector<double> quat = { // w, x, y, z
        0.999390925644008,
        -0.008357784211309,
        -0.004638522929725,
        -0.033562021520099
    };

    auto rot = quaternion_matrix(quat);

    std::cout << std::fixed << std::setprecision(8);
    for (const auto& row : rot) {
        for (double val : row) {
            std::cout << std::setw(12) << val << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
