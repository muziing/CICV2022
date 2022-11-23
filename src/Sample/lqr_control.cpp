#include "lqr_control.h"


lqrControl::lqrControl()
{
    // �����ٶ�
    vx = 0.1;
    // �����ٶ�
    vy = 0;

    //���Ĳ�ƫ����Ϊ����
    cf = -88539.01;
    // ��̥��ƫ�ն�
    cr = -88539.01;

    m = 240.1;

    // ���������ٶ�
    max_lateral_acceleration = 2.0;
    // ����ƶ����ٶ�
    standstill_acceleration = -3.0;
    // ���
    wheel_base = 1.55;
    // ǰ�����ĵ����ĵľ���
    a = 0.855;
    // �������ĵ����ĵľ���
    b = 0.695;

    // ������z��ת����ת������
    Iz = 1110.9;

    // ��̥���ת��(rad)����ʱû����
    wheel_max_degree = 0.6;
}


double lqrControl::calculateCmd(const std::vector<RefPoint>& targetPath, PanoSimSensorBus::Lidar_ObjList_G* pLidar, 
    PanoSimBasicsBus::Ego* pEgo) {
	this->vx = -pLidar->items->OBJ_Ego_Vx;
    this->vy = sqrt(abs(pow(pEgo->speed, 2) - pow(pLidar->items->OBJ_Ego_Vx, 2)));
    std::cout << "vehicle_cor vy: " << this->vy << std::endl;
	// �����ٷ���
	std::vector<std::pair<double, double>> trj_point_array;
	for (auto& Point : targetPath) {
		trj_point_array.emplace_back(Point.x, Point.y);
	}
	std::vector<double> trj_thetas;
	for (auto& Point : targetPath) {
		trj_thetas.push_back(Point.theta);
	}
	std::vector<double> trj_kappas;
	for (auto& Point : targetPath) {
		trj_kappas.push_back(Point.kappa);
	}
	// ������꣬�ʳ���λ��Ϊ(0,0)
	double currentPositionX = 0;
	double currentPositionY = 0;
	double car_yaw = 0;// ��������ϵ���ò���
	double out_angle = theta_angle(trj_point_array, trj_thetas, trj_kappas, currentPositionX, currentPositionY, car_yaw);
    return out_angle;
}

double lqrControl::theta_angle(const std::vector<std::pair<double, double>>& trj_point_array, std::vector<double>& trj_thetas,
	std::vector<double>& trj_kappas, double currentPositionX, double currentPositionY, double car_yaw)
{

	std::array<double, 5> err_k = cal_err_k(trj_point_array, trj_thetas, trj_kappas, currentPositionX, currentPositionY, car_yaw);
	Eigen::Matrix<double, 1, 4> k = cal_k(err_k);

	double forword_angle = cal_forword_angle(k, err_k);
	double angle = cal_angle(k, forword_angle, err_k);
	return angle;
}


std::array<double, 5> lqrControl::cal_err_k(const std::vector<std::pair<double, double>>& trj_point_array, 
    std::vector<double>& trj_thetas, std::vector<double>& trj_kappas, double current_post_x, 
    double current_post_y, double car_yaw)
{
    //current_post_x = current_post_x + this->vx * 0.2;// Ԥ��ģ��
    //current_post_y = current_post_y + this->vy * 0.2;
    std::array<double, 5> err_k;
    int index = 0;
    double min_dis = (std::numeric_limits<int>::max)();
    for (int i = 0; i < trj_point_array.size(); ++i) {
        double dis = pow(trj_point_array[i].first, 2) + pow(trj_point_array[i].second, 2);
        if (dis < min_dis) {
            
            min_dis = dis;
            index = i;
        }
    }
    std::cout << index << "_xy: " << trj_point_array[index].first << "  " << trj_point_array[index].second << std::endl;
    // �ҵ�index�󣬿�ʼ���ͶӰ��
    Eigen::Matrix<double, 2, 1> tor;
    tor << cos(trj_thetas[index]), sin(trj_thetas[index]);
    // Eigen::Vector2f nor;
    Eigen::Matrix<double, 2, 1> nor;
    nor << -sin(trj_thetas[index]), cos(trj_thetas[index]);

    // Eigen::Vector2f d_err;
    Eigen::Matrix<double, 2, 1> d_err;
    d_err << current_post_x - trj_point_array[index].first, current_post_y - trj_point_array[index].second;
    double phi = 0;

    // nor.transpose()��norת��
    double ed = nor.transpose() * d_err;
    
    std::cout << "���� " << ed << std::endl;

    double es = tor.transpose() * d_err;
    std::cout << "���� " << es << std::endl;

    // ͶӰ���threat�Ƕ�
    double projection_point_threat = trj_thetas[index] + trj_kappas[index] * es;

    // double phi = trj_thetas[index];
    double ed_d = vy * cos(phi - projection_point_threat) +
        vx * sin(phi - projection_point_threat);
    // ����ephi
     double ephi = sin(phi - projection_point_threat);
    //double ephi = phi - projection_point_threat;

    // ����s_d
    double s_d = (vx * cos(phi - projection_point_threat) -
        vy * sin(phi - projection_point_threat)) /
        (1 - trj_kappas[index] * ed);
    double phi_d = vx * trj_kappas[index];
    double ephi_d = phi_d - trj_kappas[index] * s_d;

    // ����ͶӰ������k
    double projection_point_curvature = trj_kappas[index];

    err_k[0] = ed;
    err_k[1] = ed_d;
    err_k[2] = ephi;
    err_k[3] = ephi_d;
    err_k[4] = projection_point_curvature;

    return err_k;
}


Eigen::Matrix<double, 1, 4> lqrControl::cal_k(std::array<double, 5> err_k)
{
    Eigen::Matrix4d A;
    A << 0, 1, 0, 0,
        0, (cf + cr) / (m * vx), -(cf + cr) / m, (a * cf - b * cr) / (m * vx),
        0, 0, 0, 1,
        0, (a * cf - b * cr) / (Iz * vx), -(a * cf - b * cr) / Iz, (a * a * cf + b * b * cr) / (Iz * vx);

    // Eigen::Vector4f B;
    Eigen::Matrix<double, 4, 1> B;
    B << 0, -cf / m, 0, -a * cf / Iz;

    // // ���óɵ�λ����
    Eigen::Matrix4d Q;
     Q.setIdentity(4, 4);
    /*Q(0, 0) = 60;
    Q(1, 1) = 1;
    Q(2, 2) = 1;
    Q(3, 3) = 1;*/
    Q(0, 0) = 25;// ֵԽ�����̰ڵ�Խ����
    Q(1, 1) = 3;// ֵԽ��������Խ��
    Q(2, 2) = 10;
    Q(3, 3) = 4;

    Eigen::Matrix<double, 1, 1> R;
    /*R(0, 0) = 35.0;  100*/
    R(0, 0) = 35.0;
    // MatrixXd����ֻ����(),VectorXd��������()������[]
    Eigen::Matrix<double, 1, 4> k = cal_dlqr(A, B, Q, R);

    return k;
}


Eigen::Matrix<double, 1, 4> lqrControl::cal_dlqr(Eigen::Matrix4d A, Eigen::Matrix<double, 4, 1> B,
    Eigen::Matrix4d Q, Eigen::Matrix<double, 1, 1> R)
{
    // �������ѭ����������
    int numLoop = 200;
    // ����Ŀ�꼫Сֵ
    double minValue = 10e-10;
    Eigen::Matrix4d p_old;
    p_old = Q;// pȡ��ֵ

    // ��ɢ��״̬����
    double ts = 0.001;
    Eigen::Matrix4d eye;
    eye.setIdentity(4, 4);

    Eigen::Matrix4d Ad;
    Ad = (eye - ts * 0.5 * A).inverse() * (eye + ts * 0.5 * A);
    Eigen::Matrix<double, 4, 1> Bd;
    Bd = B * ts;
    for (int i = 0; i < numLoop; i++)
    {
        // B.inverse()����
        Eigen::Matrix4d p_new = Ad.transpose() * p_old * Ad -
            Ad.transpose() * p_old * Bd *
            (R + Bd.transpose() * p_old * Bd).inverse() *
            Bd.transpose() * p_old * Ad +
            Q;
        // p.determinant()������ʽ
        // if (std::abs((p_old - p_new).determinant()) <= minValue) {
        // cwiseAbs()�����ֵ��maxCoeff()�����ϵ��
        if (fabs((p_new - p_old).maxCoeff()) < minValue)
        {
            p_old = p_new;
            break;
        }
        p_old = p_new;
    }
    Eigen::Matrix<double, 1, 4> k;
    // Eigen::RowVector4f;
    // ������������Χ�ĸ���������INF����������ʱ�����������ΪNaN��
    k = (R + Bd.transpose() * p_old * Bd).inverse() * Bd.transpose() * p_old * Ad;
    return k;
}

double lqrControl::cal_forword_angle(Eigen::Matrix<double, 1, 4> k,
    std::array<double, 5> err_k)
{
    double k3 = k[2];
    // ����ת��ϵ��
    double kv = a * m / (cr * wheel_base) - b * m / (cf * wheel_base);

    //ͶӰ�������final_path.k[index]
    double point_curvature = err_k[4];
    double forword_angle =
        1.0 * (wheel_base * point_curvature + kv * vx * vx * point_curvature -
        k3 * (b * point_curvature + a * m * vx * vx * point_curvature / cr / (a + b)));
    return forword_angle;
}

double lqrControl::cal_angle(Eigen::Matrix<double, 1, 4> k, double forword_angle,
    std::array<double, 5> err_k)
{
    Eigen::Matrix<double, 4, 1> err;
    err << err_k[0], err_k[1], err_k[2], err_k[3];
    double angle = (-k * err + forword_angle) * 180 * 3.67 / M_PI;
    
    if (angle > 135) {
        angle = 135;
    }
    else if (angle < -135) {
        angle = -135;
    }
    return angle;
}
