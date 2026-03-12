/**
 * @file Mesh2DQuality.cpp
 * @brief 二维网格单元质量计算实现 (无第三方依赖)
 */

#include "Mesh2DQuality.h"
#include <cmath>
#include <algorithm>

namespace MeMeshProcess {
namespace Quality2D {

namespace { // 匿名命名空间用于隐藏内部实现，避免与外部 Vector3 冲突

    constexpr double DBL_MIN_VAL = 2.2204460492503131e-15;
    constexpr double DBL_MAX_VAL = 1.0e+30;
    constexpr double PI_VAL      = 3.14159265358979323846;
    constexpr double SQRT2       = 1.41421356237309504880;
    constexpr double SQRT3       = 1.73205081001472731392;

    struct InternalVec3 {
        double x, y, z;
        InternalVec3(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {}
        InternalVec3(const double* p1, const double* p2) : x(p2[0]-p1[0]), y(p2[1]-p1[1]), z(p2[2]-p1[2]) {}

        InternalVec3 operator+(const InternalVec3& v) const { return InternalVec3(x+v.x, y+v.y, z+v.z); }
        InternalVec3 operator-(const InternalVec3& v) const { return InternalVec3(x-v.x, y-v.y, z-v.z); }
        InternalVec3 operator-() const { return InternalVec3(-x, -y, -z); }
        InternalVec3 operator*(double s) const { return InternalVec3(x*s, y*s, z*s); }
        InternalVec3 operator/(double s) const { return InternalVec3(x/s, y/s, z/s); }

        double dot(const InternalVec3& v) const { return x*v.x + y*v.y + z*v.z; }
        InternalVec3 cross(const InternalVec3& v) const {
            return InternalVec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
        }
        double lengthSq() const { return x*x + y*y + z*z; }
        double length() const { return std::sqrt(lengthSq()); }

        double normalize() {
            double len = length();
            if (len > DBL_MIN_VAL) {
                x /= len; y /= len; z /= len;
            }
            return len;
        }

        double interiorAngle(const InternalVec3& other) const {
            double len1 = length();
            double len2 = other.length();
            if (len1 <= DBL_MIN_VAL || len2 <= DBL_MIN_VAL) return 0.0;
            double cosTheta = dot(other) / (len1 * len2);
            cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
            return std::acos(cosTheta) * 180.0 / PI_VAL; // 返回角度
        }
    };

    // 为安全边界截断设计
    double safeMinMax(double val, bool isAreaRelated = false) {
        if (std::isnan(val)) return 0.0; // 防止 NaN
        if (val > 0) return std::min(val, DBL_MAX_VAL);
        return isAreaRelated ? std::max(val, -DBL_MAX_VAL) : std::max(val, -DBL_MAX_VAL); // 某些指标可能为负
    }

    void makeQuadEdges(InternalVec3 edges[4], const double coords[][3]) {
        edges[0] = InternalVec3(coords[0], coords[1]);
        edges[1] = InternalVec3(coords[1], coords[2]);
        edges[2] = InternalVec3(coords[2], coords[3]);
        edges[3] = InternalVec3(coords[3], coords[0]);
    }

    void quadSignedCornerAreas(double areas[4], const double coords[][3]) {
        InternalVec3 edges[4];
        makeQuadEdges(edges, coords);

        InternalVec3 cornerNormals[4] = {
            edges[3].cross(edges[0]),
            edges[0].cross(edges[1]),
            edges[1].cross(edges[2]),
            edges[2].cross(edges[3])
        };

        InternalVec3 principalAxes[2] = {
            edges[0] - edges[2],
            edges[1] - edges[3]
        };

        InternalVec3 unitCenterNormal = principalAxes[0].cross(principalAxes[1]);
        unitCenterNormal.normalize();

        for (int i = 0; i < 4; ++i) {
            areas[i] = unitCenterNormal.dot(cornerNormals[i]);
        }
    }
} // end anonymous namespace

// ==========================================
// Triangle Metrics
// ==========================================

double triArea(const double coords[][3]) {
    InternalVec3 side1(coords[0], coords[1]);
    InternalVec3 side3(coords[0], coords[2]);
    InternalVec3 crossVec = side1.cross(side3);
    double area = 0.5 * crossVec.length();
    return safeMinMax(area, true);
}

double triAspectRatio(const double coords[][3]) {
    InternalVec3 v[3] = {
        InternalVec3(coords[0], coords[1]),
        InternalVec3(coords[1], coords[2]),
        InternalVec3(coords[2], coords[0])
    };

    double a1 = v[0].length();
    double b1 = v[1].length();
    double c1 = v[2].length();

    double hm = std::max({a1, b1, c1});
    InternalVec3 ab = v[0].cross(v[1]);
    double denominator = ab.length();

    if (denominator < DBL_MIN_VAL) return DBL_MAX_VAL;

    double aspectRatio = (SQRT3 / 6.0) * hm * (a1 + b1 + c1) / denominator;
    return safeMinMax(aspectRatio);
}

double triScaledJacobian(const double coords[][3]) {
    InternalVec3 edge[3] = {
        InternalVec3(coords[0], coords[1]),
        InternalVec3(coords[0], coords[2]),
        InternalVec3(coords[1], coords[2])
    };

    InternalVec3 first = edge[1] - edge[0];  // (p2 - p0) - (p1 - p0) -> p2 - p1 (v[2]) wait, first logic is from verdict?
    // Verdict implements: first = p2-p0 - (p1-p0) => p2-p1. No, edge[1] is p2-p0, edge[0] is p1-p0. 
    // Actually verdict tri_scaled_jacobian uses:
    // edge[0] = p1-p0, edge[1] = p2-p0, edge[2] = p2-p1
    // first = edge[1] - edge[0] = p2-p1, second = edge[2] - edge[0] = p2-p1 - (p1-p0)?
    // Let's implement simpler:
    double l0 = edge[0].length(); // p1 - p0
    double l1 = edge[1].length(); // p2 - p0
    double l2 = edge[2].length(); // p2 - p1

    InternalVec3 crossVec = edge[0].cross(edge[1]);
    double jacobian = crossVec.length();

    double maxEdgeProduct = std::max({l0 * l1, l1 * l2, l0 * l2});
    if (maxEdgeProduct < DBL_MIN_VAL) return 0.0;

    jacobian *= (2.0 / SQRT3);
    jacobian /= maxEdgeProduct;
    return safeMinMax(jacobian);
}

double triCondition(const double coords[][3]) {
    InternalVec3 v1(coords[0], coords[1]);
    InternalVec3 v2(coords[0], coords[2]);

    InternalVec3 triNormal = v1.cross(v2);
    double areaX2 = triNormal.length();

    if (areaX2 < DBL_MIN_VAL) return DBL_MAX_VAL;

    double condition = (v1.dot(v1) + v2.dot(v2) - v1.dot(v2)) / (areaX2 * SQRT3);
    return safeMinMax(condition);
}

void triAngles(const double coords[][3], double& minAngle, double& maxAngle) {
    InternalVec3 sides[4];
    sides[0] = InternalVec3(coords[0], coords[1]); // p1 - p0
    sides[1] = InternalVec3(coords[1], coords[2]); // p2 - p1
    sides[2] = InternalVec3(coords[0], coords[2]); // p2 - p0
    sides[3] = -sides[1]; // p1 - p2

    double lengthsSq[3] = { sides[0].lengthSq(), sides[1].lengthSq(), sides[2].lengthSq() };

    if (lengthsSq[0] < DBL_MIN_VAL || lengthsSq[1] < DBL_MIN_VAL || lengthsSq[2] < DBL_MIN_VAL) {
        minAngle = 0.0;
        maxAngle = 360.0;
        return;
    }

    // 找到最短边与最长边
    int shortSide = 0, longSide = 0;
    if (lengthsSq[1] < lengthsSq[0]) shortSide = 1;
    if (lengthsSq[2] < lengthsSq[shortSide]) shortSide = 2;

    if (lengthsSq[1] > lengthsSq[0]) longSide = 1;
    if (lengthsSq[2] > lengthsSq[longSide]) longSide = 2;

    // 计算最小角 (最短边对角)
    if (shortSide == 0) minAngle = sides[2].interiorAngle(sides[1]);
    else if (shortSide == 1) minAngle = sides[0].interiorAngle(sides[2]);
    else minAngle = sides[0].interiorAngle(sides[3]);

    // 计算最大角 (最长边对角)
    if (longSide == 0) maxAngle = sides[2].interiorAngle(sides[1]);
    else if (longSide == 1) maxAngle = sides[0].interiorAngle(sides[2]);
    else maxAngle = sides[0].interiorAngle(sides[3]);

    minAngle = safeMinMax(minAngle);
    maxAngle = safeMinMax(maxAngle);
}

double triEquiangleSkew(const double coords[][3]) {
    double minAngle = 0.0, maxAngle = 0.0;
    triAngles(coords, minAngle, maxAngle);

    double skewMax = (maxAngle - 60.0) / 120.0;
    double skewMin = (60.0 - minAngle) / 60.0;

    return std::max(skewMax, skewMin);
}

// ==========================================
// Quadrangle Metrics
// ==========================================

double quadArea(const double coords[][3]) {
    double cornerAreas[4];
    quadSignedCornerAreas(cornerAreas, coords);
    double area = 0.25 * (cornerAreas[0] + cornerAreas[1] + cornerAreas[2] + cornerAreas[3]);
    return safeMinMax(area, true);
}

double quadEdgeRatio(const double coords[][3]) {
    InternalVec3 edges[4];
    makeQuadEdges(edges, coords);

    double a2 = edges[0].lengthSq();
    double b2 = edges[1].lengthSq();
    double c2 = edges[2].lengthSq();
    double d2 = edges[3].lengthSq();

    double mab = std::min(a2, b2);
    double Mab = std::max(a2, b2);
    double mcd = std::min(c2, d2);
    double Mcd = std::max(c2, d2);

    double m2 = std::min(mab, mcd);
    double M2 = std::max(Mab, Mcd);

    if (m2 < DBL_MIN_VAL) return DBL_MAX_VAL;

    double edgeRatio = std::sqrt(M2 / m2);
    return safeMinMax(edgeRatio);
}

double quadAspectRatio(const double coords[][3]) {
    InternalVec3 edges[4];
    makeQuadEdges(edges, coords);

    double a1 = edges[0].length();
    double b1 = edges[1].length();
    double c1 = edges[2].length();
    double d1 = edges[3].length();

    double ma = std::max(a1, b1);
    double mb = std::max(c1, d1);
    double hm = std::max(ma, mb);

    double cornerAreas[4];
    quadSignedCornerAreas(cornerAreas, coords);

    double totalCornerArea = cornerAreas[0] + cornerAreas[1] + cornerAreas[2] + cornerAreas[3];
    if (totalCornerArea < DBL_MIN_VAL) return DBL_MAX_VAL;

    double aspectRatio = hm * (a1 + b1 + c1 + d1) / totalCornerArea;
    return safeMinMax(aspectRatio);
}

double quadWarpage(const double coords[][3]) {
    InternalVec3 edges[4];
    makeQuadEdges(edges, coords);

    InternalVec3 cornerNormals[4] = {
        edges[3].cross(edges[0]),
        edges[0].cross(edges[1]),
        edges[1].cross(edges[2]),
        edges[2].cross(edges[3])
    };

    if (cornerNormals[0].normalize() < DBL_MIN_VAL ||
        cornerNormals[1].normalize() < DBL_MIN_VAL ||
        cornerNormals[2].normalize() < DBL_MIN_VAL ||
        cornerNormals[3].normalize() < DBL_MIN_VAL) {
        return 0.0;
    }

    double warpage = std::min(cornerNormals[0].dot(cornerNormals[2]), cornerNormals[1].dot(cornerNormals[3]));
    warpage = warpage * warpage * warpage; // 立方
    return safeMinMax(warpage);
}

double quadScaledJacobian(const double coords[][3]) {
    InternalVec3 edges[4];
    makeQuadEdges(edges, coords);

    InternalVec3 n[4];
    n[0] = edges[3].cross(edges[0]);
    n[1] = edges[0].cross(edges[1]);
    n[2] = edges[1].cross(edges[2]);
    n[3] = edges[2].cross(edges[3]);

    InternalVec3 diag0 = InternalVec3(coords[0], coords[2]);
    InternalVec3 diag1 = InternalVec3(coords[1], coords[3]);
    InternalVec3 unitNormal = diag0.cross(diag1);
    unitNormal.normalize();

    double minJac = DBL_MAX_VAL;
    for (int i = 0; i < 4; ++i) {
        double l1 = edges[i].length();
        double l2 = edges[(i + 3) % 4].length();
        if (l1 * l2 < DBL_MIN_VAL) return 0.0;
        double cornerJac = unitNormal.dot(n[i]) / (l1 * l2);
        minJac = std::min(minJac, cornerJac);
    }

    return safeMinMax(minJac);
}

double quadSkew(const double coords[][3]) {
    InternalVec3 principleAxes[2] = {
        InternalVec3(coords[0], coords[1]) + InternalVec3(coords[3], coords[2]),
        InternalVec3(coords[0], coords[3]) + InternalVec3(coords[1], coords[2])
    };

    if (principleAxes[0].normalize() < DBL_MIN_VAL || principleAxes[1].normalize() < DBL_MIN_VAL) {
        return 0.0;
    }

    double skew = std::abs(principleAxes[0].dot(principleAxes[1]));
    return safeMinMax(skew);
}

double quadTaper(const double coords[][3]) {
    InternalVec3 principleAxes[2] = {
        InternalVec3(coords[0], coords[1]) + InternalVec3(coords[3], coords[2]),
        InternalVec3(coords[0], coords[3]) + InternalVec3(coords[1], coords[2])
    };

    InternalVec3 crossDerivative = (InternalVec3(coords[0], coords[1]) + InternalVec3(coords[2], coords[3])) * 0.5;

    double len0 = principleAxes[0].length();
    double len1 = principleAxes[1].length();
    double minLen = std::min(len0, len1);

    if (minLen < DBL_MIN_VAL) return DBL_MAX_VAL;

    double taper = (crossDerivative.length() * 2.0) / minLen;
    return safeMinMax(taper);
}

void quadAngles(const double coords[][3], double& minAngle, double& maxAngle) {
    InternalVec3 edges[4];
    makeQuadEdges(edges, coords);

    double length[4] = { edges[0].length(), edges[1].length(), edges[2].length(), edges[3].length() };

    if (length[0] <= DBL_MIN_VAL || length[1] <= DBL_MIN_VAL ||
        length[2] <= DBL_MIN_VAL || length[3] <= DBL_MIN_VAL) {
        minAngle = 360.0;
        maxAngle = 0.0;
        return;
    }

    minAngle = 360.0;
    maxAngle = 0.0;

    double angle = std::acos(-edges[0].dot(edges[1]) / (length[0] * length[1]));
    minAngle = std::min(angle, minAngle);
    maxAngle = std::max(angle, maxAngle);

    angle = std::acos(-edges[1].dot(edges[2]) / (length[1] * length[2]));
    minAngle = std::min(angle, minAngle);
    maxAngle = std::max(angle, maxAngle);

    angle = std::acos(-edges[2].dot(edges[3]) / (length[2] * length[3]));
    minAngle = std::min(angle, minAngle);
    maxAngle = std::max(angle, maxAngle);

    angle = std::acos(-edges[3].dot(edges[0]) / (length[3] * length[0]));
    minAngle = std::min(angle, minAngle);
    maxAngle = std::max(angle, maxAngle);

    maxAngle = maxAngle * 180.0 / PI_VAL;
    minAngle = minAngle * 180.0 / PI_VAL;

    double areas[4];
    quadSignedCornerAreas(areas, coords);

    if (areas[0] < 0 || areas[1] < 0 || areas[2] < 0 || areas[3] < 0) {
        maxAngle = 360.0 - maxAngle;
    }

    minAngle = safeMinMax(minAngle);
    maxAngle = safeMinMax(maxAngle);
}

double quadAngleHelper(const double coords[][3], bool isMax) {
    double minAngle = 0.0, maxAngle = 0.0;
    quadAngles(coords, minAngle, maxAngle);
    return isMax ? maxAngle : minAngle;
}

} // namespace Quality2D
} // namespace MeMeshProcess
