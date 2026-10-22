#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}

bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    (void)x;
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const
{
    Eigen::Map<const Eigen::Vector3d> p_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

    Eigen::Map<const Eigen::Vector3d> p_x(x);
    Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

    Eigen::Map<Eigen::Vector3d> dp(y_minus_x);
    Eigen::Map<Eigen::Vector3d> dq(y_minus_x + 3);

    dp = p_y - p_x;

    // Inverse of q' = normalize(q * deltaQ(theta)). deltaQ stores theta/2 and w = 1,
    // so theta = 2 * q_diff.vec() / q_diff.w() after the double-cover sign flip.
    Eigen::Quaterniond q_diff = q_x.inverse() * q_y;
    if (q_diff.w() < 0.0)
    {
        q_diff.coeffs() *= -1.0;
    }
    if (q_diff.w() < 1e-12)
    {
        dq = 2.0 * q_diff.vec();
    }
    else
    {
        dq = 2.0 * q_diff.vec() / q_diff.w();
    }

    return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    (void)x;
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.topLeftCorner<6, 6>().setIdentity();

    return true;
}
