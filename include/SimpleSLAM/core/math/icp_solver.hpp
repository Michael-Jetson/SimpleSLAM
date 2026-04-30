#pragma once

/// @file icp_solver.hpp
/// ICP 求解器——从 MatchResult 的残差+雅可比求解 SE(3) 位姿增量
///
/// 纯数学工具类，不依赖任何地图结构。
/// 输入：MatchResult（由 RegistrationTarget::match() 填充）
/// 输出：6D 扭量增量 δξ ∈ se(3)
///
/// 数学原理：
///   法方程 (JᵀWJ)δξ = -JᵀWr
///   其中 W = diag(w₁, w₂, ...) 是鲁棒核权重矩阵
///   逐行累加 JᵀJ 和 Jᵀr，最后 LDLT 解 6×6 系统

#include <SimpleSLAM/core/concepts/registration_target.hpp>
#include <SimpleSLAM/core/types/geometry.hpp>

#include <Eigen/Dense>

#include <cmath>

namespace simpleslam {

/// 鲁棒核函数类型
enum class RobustKernel : uint8_t {
    None,           ///< L2 损失，w=1（无鲁棒性）
    GemanMcClure,   ///< KISS-ICP 使用，强重降，外点权重趋零
};

/// ICP 求解器配置
struct IcpConfig {
    int max_iterations = 20;             ///< 最大迭代次数
    double convergence_threshold = 1e-4; ///< δξ 范数收敛阈值
    RobustKernel kernel = RobustKernel::None;
    double kernel_scale = 1.0;           ///< 鲁棒核参数 σ（自适应阈值）
};

/// ICP Gauss-Newton 求解器
///
/// 每次调用 solveOneStep() 消费一个 MatchResult，产出一个 6D 位姿增量。
/// LoIcpOdometry 在外层循环中交替调用 match() 和 solveOneStep()。
class IcpSolver final {
public:
    explicit IcpSolver(const IcpConfig& config = {}) : config_(config) {}

    /// 从 MatchResult 求解一步位姿增量 δξ
    ///
    /// MatchResult 内存布局（点到点模式，每个匹配贡献 3 行）：
    ///   residuals:  [r₀, r₁, r₂, r₃, r₄, r₅, ...]   每个 rᵢ 是标量
    ///   jacobians:  [J₀(6个), J₁(6个), J₂(6个), ...]  每行 6 个 double
    ///   num_valid:  总行数（= 匹配点数 × 3）
    ///
    /// 数学：逐行累加法方程 H δξ = -b
    ///   H = Σ wᵢ Jᵢᵀ Jᵢ  (6×6)
    ///   b = Σ wᵢ Jᵢᵀ rᵢ  (6×1)
    ///   δξ = H⁻¹(-b)     (LDLT 求解)
    Eigen::Matrix<double, 6, 1> solveOneStep(const MatchResult& result) {
        // 累加器初始化为零
        Eigen::Matrix<double, 6, 6> JtJ = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> Jtr = Eigen::Matrix<double, 6, 1>::Zero();

        // 逐行累加——每行一个标量残差 + 一行 1×6 雅可比
        for (int i = 0; i < result.num_valid; ++i) {
            // 取第 i 行的标量残差
            double ri = result.residuals[i];

            // 零拷贝映射第 i 行的 1×6 雅可比
            Eigen::Map<const Eigen::Matrix<double, 1, 6>> Ji(
                result.jacobians.data() + i * 6);

            // 鲁棒核权重：小残差 w≈1，大残差（外点）w→0
            double wi = robustWeight(ri * ri);

            // 累加到法方程
            //   JᵢᵀJᵢ 是 6×6 的秩 1 矩阵（6×1 乘 1×6）
            //   Jᵢᵀrᵢ 是 6×1 向量（6×1 乘标量）
            JtJ.noalias() += wi * Ji.transpose() * Ji;
            Jtr.noalias() += wi * (Ji.transpose() * ri);
        }

        // LDLT 解 6×6 对称系统（允许半正定——退化场景安全）
        return JtJ.ldlt().solve(-Jtr);
    }

    [[nodiscard]] const IcpConfig& config() const { return config_; }
    void setConfig(const IcpConfig& config) { config_ = config; }

private:
    /// 鲁棒核权重——从残差平方映射到 [0, 1] 的权重
    ///
    /// None:         w = 1（所有点等权）
    /// GemanMcClure: w = σ² / (σ + r²)²（KISS-ICP 的重参数化形式）
    [[nodiscard]] double robustWeight(double residual_sq) const {
        switch (config_.kernel) {
        case RobustKernel::GemanMcClure: {
            double s = config_.kernel_scale;
            double denom = s + residual_sq;
            return (s * s) / (denom * denom);
        }
        default:
            return 1.0;
        }
    }

    IcpConfig config_;
};

}  // namespace simpleslam
