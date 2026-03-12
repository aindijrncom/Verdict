#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "Mesh2DQuality.h"
#include "verdict.h"

using namespace MeMeshProcess::Quality2D;
using namespace verdict;

void print_compare(const char* name, double v1, double v2) {
    double diff = std::abs(v1 - v2);
    std::cout << std::left << std::setw(25) << name 
              << " | Verdict: " << std::setw(12) << v1 
              << " | Hand: " << std::setw(12) << v2 
              << " | Diff: " << diff << (diff < 2e-7 ? " [OK]" : " [FAIL]") << std::endl;
}

void test_triangle() {
    std::cout << "\n--- Testing Triangle ---" << std::endl;
    // 普通三角形
    double tri_coords[3][3] = {
        {0.0, 0.0, 0.0},
        {1.1, 0.2, 0.0},
        {0.4, 0.9, 0.0}
    };

    print_compare("Tri Area", tri_area(3, tri_coords), triArea(tri_coords));
    print_compare("Tri Aspect Ratio", tri_aspect_ratio(3, tri_coords), triAspectRatio(tri_coords));
    print_compare("Tri Scaled Jacobian", tri_scaled_jacobian(3, tri_coords), triScaledJacobian(tri_coords));
    print_compare("Tri Condition", tri_condition(3, tri_coords), triCondition(tri_coords));
    print_compare("Tri Equiangle Skew", tri_equiangle_skew(3, tri_coords), triEquiangleSkew(tri_coords));

    double triMinV, triMaxV;
    triAngles(tri_coords, triMinV, triMaxV);
    print_compare("Tri Min Angle", tri_minimum_angle(3, tri_coords), triMinV);
    print_compare("Tri Max Angle", tri_maximum_angle(3, tri_coords), triMaxV);
}

void test_quad() {
    std::cout << "\n--- Testing Quad ---" << std::endl;
    // 稍微带点畸形的四边形
    double quad_coords[4][3] = {
        {0.0, 0.0, 0.0},
        {1.2, 0.1, 0.2},
        {0.9, 1.1, -0.1},
        {-0.1, 1.0, 0.0}
    };

    print_compare("Quad Area", quad_area(4, quad_coords), quadArea(quad_coords));
    print_compare("Quad Edge Ratio", quad_edge_ratio(4, quad_coords), quadEdgeRatio(quad_coords));
    print_compare("Quad Aspect Ratio", quad_aspect_ratio(4, quad_coords), quadAspectRatio(quad_coords));
    print_compare("Quad Warpage", quad_warpage(4, quad_coords), quadWarpage(quad_coords));
    print_compare("Quad Scaled Jacobian", quad_scaled_jacobian(4, quad_coords), quadScaledJacobian(quad_coords));
    print_compare("Quad Skew", quad_skew(4, quad_coords), quadSkew(quad_coords));
    print_compare("Quad Taper", quad_taper(4, quad_coords), quadTaper(quad_coords));
    
    double minV, maxV;
    quadAngles(quad_coords, minV, maxV);
    print_compare("Quad Min Angle", quad_minimum_angle(4, quad_coords), minV);
    print_compare("Quad Max Angle", quad_maximum_angle(4, quad_coords), maxV);
}

int main() {
    std::cout << std::fixed << std::setprecision(8);
    test_triangle();
    test_quad();
    
    std::cout << "\nTest Finished. Press Enter to exit..." << std::endl;
    std::cin.get(); 
    return 0;
}
