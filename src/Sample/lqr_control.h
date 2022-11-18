#ifndef __LQR_CONTROL__
#define __LQR_CONTROL__

#pragma once

#include "control.h"

class lqrControl : public control
{
public:
    lqrControl();
    ~lqrControl() = default;

    double calculateCmd(const std::vector<RefPoint>& targetPath, PanoSimSensorBus::Lidar_ObjList_G* pLidar) override;

    // ����ǰ��ת��
    double theta_angle(const std::vector<std::pair<double, double>>& trj_point_array, std::vector<double>& trj_thetas,
        std::vector<double>& trj_kappas, double currentPositionX, double currentPositionY, double car_yaw);

    // ������� ed ed' ephi ephi'
    std::array<double, 5> cal_err_k(const std::vector<std::pair<double, double>>& trj_point_array, std::vector<double>& trj_thetas, 
        std::vector<double>& trj_kappas, double current_post_x, double current_post_y, double car_yaw);

    // ����lqr��k1 k2 k3 k4
    Eigen::Matrix<double, 1, 4> cal_k(std::array<double, 5> err_k);

    // ����dlqr
    Eigen::Matrix<double, 1, 4> cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double, 4, 1> B,
        Eigen::Matrix4d Q, Eigen::Matrix<double, 1, 1> R);

    // ����ǰ��
    double cal_forword_angle(Eigen::Matrix<double, 1, 4> k, std::array<double, 5> err_k);

    // ����Ƕ�
    double cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle, std::array<double, 5> err_k);


private:
    // �����ٶ�
    double vx;
    // �����ٶ�
    double vy;
    // ��̥��ƫ�ն�
    double cf, cr;

    // ǰ�������غ�
    double m;

    // ���������ٶ�
    double max_lateral_acceleration;
    // ����ƶ����ٶ�
    double standstill_acceleration;
    // ���
    double wheel_base;
    // ǰ�����ĵ����ĵľ���
    double a;
    // �������ĵ����ĵľ���
    double b;

    // ������z��ת����ת������
    double Iz;

    // ��̥���ת��(rad)
    double wheel_max_degree;
};



#endif __LQR_CONTROL__