/**
 * @file Mesh2DQuality.h
 * @brief 二维网格单元 (Triangle, Quadrangle) 质量计算库 
 * @details 基于零依赖原则，提供独立于 Verdict 库的高性能网格计算接口，
 *          适配 MeGrid 内置的数据结构风格。
 */

#ifndef MESH2DQUALITY_H
#define MESH2DQUALITY_H

#include <MeMeshProcessDll.h>

namespace MeMeshProcess {
namespace Quality2D {

    /**
     * @brief 三角形单元质量指标常量声明
     */
    constexpr double MIN_TRI_AREA = 1e-12; // 处理面积时的除零保护容差

    /**
     * @brief 计算三角形面积 (Triangle Area)
     * @param coords 三个节点的坐标数组 [3][3]
     * @return 面积值，若退化则返回 0.0
     */
    ME_MESHPROCESS_API double triArea(const double coords[][3]);

    /**
     * @brief 计算三角形长宽比 (Triangle Aspect Ratio)
     * @param coords 三个节点的坐标数组 [3][3]
     */
    ME_MESHPROCESS_API double triAspectRatio(const double coords[][3]);

    /**
     * @brief 计算三角形缩放雅可比 (Triangle Scaled Jacobian)
     * @param coords 三个节点的坐标数组 [3][3]
     */
    ME_MESHPROCESS_API double triScaledJacobian(const double coords[][3]);

    /**
     * @brief 计算三角形条件数 (Triangle Condition)
     * @param coords 三个节点的坐标数组 [3][3]
     */
    ME_MESHPROCESS_API double triCondition(const double coords[][3]);

    /**
     * @brief 计算三角形最小和最大内角 (单位：度)
     * @param coords 三个节点的坐标数组 [3][3]
     * @param minAngle 输出参数，返回最小角度
     * @param maxAngle 输出参数，返回最大角度
     */
    ME_MESHPROCESS_API void triAngles(const double coords[][3], double& minAngle, double& maxAngle);

    /**
     * @brief 计算三角形等角偏斜度 (Triangle Equiangle Skew)
     * @param coords 三个节点的坐标数组 [3][3]
     */
    ME_MESHPROCESS_API double triEquiangleSkew(const double coords[][3]);

    /**
     * @brief 四边形单元质量指标常量声明
     */
    constexpr double MIN_QUAD_AREA = 1e-12;

    /**
     * @brief 计算四边形面积 (Quad Area)
     * @param coords 四个节点的坐标数组 [4][3]
     */
    ME_MESHPROCESS_API double quadArea(const double coords[][3]);

    /**
     * @brief 计算四边形边长比 (Quad Edge Ratio)
     * @param coords 四个节点的坐标数组 [4][3]
     */
    ME_MESHPROCESS_API double quadEdgeRatio(const double coords[][3]);

    /**
     * @brief 计算四边形长宽比 (Quad Aspect Ratio)
     * @param coords 四个节点的坐标数组 [4][3]
     */
    ME_MESHPROCESS_API double quadAspectRatio(const double coords[][3]);

    /**
     * @brief 计算四边形扭曲度 (Quad Warpage)
     * @details 衡量四边形偏离严格平面的程度
     * @param coords 四个节点的坐标数组 [4][3]
     */
    ME_MESHPROCESS_API double quadWarpage(const double coords[][3]);

    /**
     * @brief 计算四边形缩放雅可比 (Quad Scaled Jacobian)
     * @param coords 四个节点的坐标数组 [4][3]
     */
    ME_MESHPROCESS_API double quadScaledJacobian(const double coords[][3]);

    /**
     * @brief 计算四边形扭斜度 (Quad Skew)
     * @param coords 四个节点的坐标数组 [4][3]
     */
    ME_MESHPROCESS_API double quadSkew(const double coords[][3]);

    /**
     * @brief 计算四边形锥度 (Quad Taper)
     * @param coords 四个节点的坐标数组 [4][3]
     */
    ME_MESHPROCESS_API double quadTaper(const double coords[][3]);

    /**
     * @brief 计算四边形最小和最大内角 (单位：度)
     * @param coords 四个节点的坐标数组 [4][3]
     * @param minAngle 输出参数，返回最小角
     * @param maxAngle 输出参数，返回最大角
     */
    ME_MESHPROCESS_API void quadAngles(const double coords[][3], double& minAngle, double& maxAngle);

    /**
     * @brief 计算四边形最小和最大内角的辅助包装版本
     * @param coords 四个节点的坐标数组 [4][3]
     * @param isMax 是否返回最大角，反之返回最小角
     */
    ME_MESHPROCESS_API double quadAngleHelper(const double coords[][3], bool isMax);

} // namespace Quality2D
} // namespace MeMeshProcess

#endif // MESH2DQUALITY_H
