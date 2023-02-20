// Kalman filter implementation using Eigen.
// @author: Shangjie Li
// @date: 2021.03.30

#pragma once

#include <Eigen/Dense>

class KalmanFilter2D
{
private:
    // 时间间隔
    double ti_;

    // 过程噪声、量测噪声
    Eigen::Matrix<double, 2, 2> noise_q_;
    Eigen::Matrix<double, 1, 1> noise_r_;

    // 状态向量、状态协方差矩阵
    Eigen::Matrix<double, 2, 1> xx_;
    Eigen::Matrix<double, 2, 2> pp_;

public:
    // 初始化KalmanFilter2D对象
    // 匀速直线运动模型
    // 二维状态向量
    void init(double time_interval,
                   double x,
                   double dx,
                   double sigma_ax = 1,
                   double sigma_ox = 1);

    // 计算KF预测方程
    void predict();

    // 计算KF更新方程
    void update(double zx);

    // 返回状态向量、状态协方差矩阵
    Eigen::Matrix<double, 2, 1> get_state();
    Eigen::Matrix<double, 2, 2> get_covariance();

};

class KalmanFilter4D
{
private:
    // 时间间隔
    double ti_;

    // 过程噪声、量测噪声
    Eigen::Matrix<double, 4, 4> noise_q_;
    Eigen::Matrix<double, 2, 2> noise_r_;

    // 状态向量、状态协方差矩阵
    Eigen::Matrix<double, 4, 1> xx_;
    Eigen::Matrix<double, 4, 4> pp_;

public:
    // 初始化KalmanFilter4D对象
    // 匀速直线运动模型
    // 四维状态向量
    void init(double time_interval,
                   double x,
                   double dx,
                   double y,
                   double dy,
                   double sigma_ax = 1,
                   double sigma_ay = 1,
                   double sigma_ox = 1,
                   double sigma_oy = 1);

    // 计算KF预测方程
    void predict();

    // 计算KF更新方程
    void update(double zx, double zy);

    // 返回状态向量、状态协方差矩阵
    Eigen::Matrix<double, 4, 1> get_state();
    Eigen::Matrix<double, 4, 4> get_covariance();

    // 计算新息加权范数
    double compute_the_residual(double zx, double zy);

    // 计算跟踪门
    // 椭圆方程为x^2 / a^2 + y^2 / b^2 = 1
    Eigen::Matrix<double, 2, 1> compute_association_gate(double g);

};

void KalmanFilter2D::init(double time_interval,
                               double x,
                               double dx,
                               double sigma_ax,
                               double sigma_ox)
{
    ti_ = time_interval;

    Eigen::Matrix<double, 2, 1> gg;
    gg << 0.5 * ti_ * ti_, ti_;

    Eigen::Matrix<double, 1, 1> q;
    q << sigma_ax * sigma_ax;
    noise_q_ = gg * q * gg.transpose();

    Eigen::Matrix<double, 1, 1> r;
    r << sigma_ox * sigma_ox;
    noise_r_ = r;

    // 初始化xx_和pp_
    xx_ << x, dx;
    pp_ << sigma_ox * sigma_ox, 0, 0, 0;
}

void KalmanFilter2D::predict()
{
    Eigen::Matrix<double, 2, 2> ff;
    ff << 1, ti_, 0, 1;

    // 预测xx_和pp_
    xx_ = ff * xx_;
    pp_ = ff * pp_ * ff.transpose() + noise_q_;
}

void KalmanFilter2D::update(double zx)
{
    Eigen::Matrix<double, 1, 1> zs;
    zs << zx;

    Eigen::Matrix<double, 1, 2> hh;
    hh << 1, 0;

    Eigen::Matrix<double, 1, 1> zz;
    zz = zs - hh * xx_;

    Eigen::Matrix<double, 1, 1> ss;
    ss = hh * pp_ * hh.transpose() + noise_r_;

    Eigen::Matrix<double, 2, 1> kk;
    kk = pp_ * hh.transpose() * ss.inverse();

    // 更新xx_和pp_
    xx_ = xx_ + kk * zz;
    pp_ = pp_ - kk * hh * pp_;
}

Eigen::Matrix<double, 2, 1> KalmanFilter2D::get_state() {return xx_;}

Eigen::Matrix<double, 2, 2> KalmanFilter2D::get_covariance() {return pp_;}

void KalmanFilter4D::init(double time_interval,
                               double x,
                               double dx,
                               double y,
                               double dy,
                               double sigma_ax,
                               double sigma_ay,
                               double sigma_ox,
                               double sigma_oy)
{
    ti_ = time_interval;

    Eigen::Matrix<double, 4, 2> gg;
    gg << 0.5 * ti_ * ti_, 0, ti_, 0, 0, 0.5 * ti_ * ti_, 0, ti_;

    Eigen::Matrix<double, 2, 2> q;
    q << sigma_ax * sigma_ax, 0, 0, sigma_ay * sigma_ay;
    noise_q_ = gg * q * gg.transpose();

    Eigen::Matrix<double, 2, 2> r;
    r << sigma_ox * sigma_ox, 0, 0, sigma_oy * sigma_oy;
    noise_r_ = r;

    // 初始化xx_和qq_
    xx_ << x, dx, y, dy;
    pp_ << sigma_ox * sigma_ox, 0, 0, 0, 0, 0, 0, 0, 0, 0, sigma_oy * sigma_oy, 0, 0, 0, 0, 0;
}

void KalmanFilter4D::predict()
{
    Eigen::Matrix<double, 4, 4> ff;
    ff << 1, ti_, 0, 0, 0, 1, 0, 0, 0, 0, 1, ti_, 0, 0, 0, 1;

    // 预测xx_和pp_
    xx_ = ff * xx_;
    pp_ = ff * pp_ * ff.transpose() + noise_q_;
}

void KalmanFilter4D::update(double zx, double zy)
{
    Eigen::Matrix<double, 2, 1> zs;
    zs << zx, zy;

    Eigen::Matrix<double, 2, 4> hh;
    hh << 1, 0, 0, 0, 0, 0, 1, 0;

    Eigen::Matrix<double, 2, 1> zz;
    zz = zs - hh * xx_;

    Eigen::Matrix<double, 2, 2> ss;
    ss = hh * pp_ * hh.transpose() + noise_r_;

    Eigen::Matrix<double, 4, 2> kk;
    kk = pp_ * hh.transpose() * ss.inverse();

    // 更新xx_和pp_
    xx_ = xx_ + kk * zz;
    pp_ = pp_ - kk * hh * pp_;
}

Eigen::Matrix<double, 4, 1> KalmanFilter4D::get_state() {return xx_;}

Eigen::Matrix<double, 4, 4> KalmanFilter4D::get_covariance() {return pp_;}

double KalmanFilter4D::compute_the_residual(double zx, double zy)
{
    Eigen::Matrix<double, 2, 1> zs;
    zs << zx, zy;

    Eigen::Matrix<double, 2, 4> hh;
    hh << 1, 0, 0, 0, 0, 0, 1, 0;

    Eigen::Matrix<double, 2, 1> zz;
    zz = zs - hh * xx_;

    Eigen::Matrix<double, 2, 2> ss;
    ss = hh * pp_ * hh.transpose() + noise_r_;

    double dd = zz.transpose() * ss.inverse() * zz;
    return dd;
}

Eigen::Matrix<double, 2, 1> KalmanFilter4D::compute_association_gate(double g)
{
    double a, b;
    a = sqrt(g * (pp_(0, 0) + noise_r_(0, 0)));
    b = sqrt(g * (pp_(2, 2) + noise_r_(1, 1)));

    Eigen::Matrix<double, 2, 1> ab;
    ab << a, b;
    return ab;
}
