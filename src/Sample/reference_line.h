#ifndef reference_line_
#define reference_line_
#pragma once

#include <iostream>
#include <vector>
#include <deque>
#include <cmath>

#include <Eigen/Eigen>

#include <SensorBusDef.h>
#include <BasicsBusDef.h>

struct Point2d_s {
  double x;
  double y;
};

struct RefPoint {
  double x;
  double y;
  double kappa;
  double theta;
};

class ReferenceLine {
 public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;

  /**
   * @brief 按颜色分别存储所有锥桶的坐标数据
   *
   * @param pLidar 目标级激光雷达传感器返回的数据
   */
  void LoadTrafficConeData(PanoSimSensorBus::Lidar_ObjList_G *pLidar);

  /**
   * @brief 计算并存储所有中心点（未排序）
   */
  void CalcCenterPoints();

  /**
   * @brief 对所有中心点按路径顺序排序
   */
  void SortCenterPoints();

  /**
 * @brief 计算 Kappa 与 theta,并保存至 path_points 中
 */
  void CalcKappaTheta();

  /// 一个迷之又迷、从未被调用过的函数
  void GetKappa(std::vector<std::pair<double, double>> final_center_point_xy);

  std::vector<std::pair<double, double>> GetCenterPointXySort() {
	return this->center_points_xy_sorted;
  }

  std::vector<std::pair<double, double>> GetCenterPointXyFinal() {
	return this->center_points_xy_final;
  }

  /// 返回所有起终点黄色锥桶的(x, y)坐标
  std::vector<std::pair<double, double>> GetYellowPoints() {
	return this->yellow_xy;
  }
  void SetCenterPointXyFinal(const std::vector<std::pair<double, double>> &center_points) {
	this->center_points_xy_final = center_points;  // FIXME 在进行插值计算之后自动设置，而不是显式调用此函数
  }

 public:
  std::vector<RefPoint> path_points;  // 最终输出的点 x, y, kappa, theta

 private:
  int RefPointCounter{};
  size_t inner = 0;  // 内侧锥桶个数
  size_t outer = 0;  // 外侧锥桶个数
  std::vector<std::pair<double, double>> center_point_xy;  // 所有中心点的坐标
  std::vector<std::pair<double, double>> center_points_xy_sorted;  // 保存排序后的所有中心点坐标
  std::vector<std::pair<double, double>> center_points_xy_final;  // 插值后的所有中心点坐标
  std::vector<std::pair<double, double>> in_xy;  // 存储所有右侧蓝色锥桶的(x, y)坐标
  std::vector<std::pair<double, double>> out_xy;  // 存储所有左侧红色锥桶的(x, y)坐标
  std::vector<std::pair<double, double>> yellow_xy;  // 存储所有起终点黄色锥桶的(x, y)坐标
  std::vector<RefPoint> RefMsg; // FIXME 貌似没用用上，干什么用的？
};

/// 辅助函数，用于计算曲率 kappa
double CalculateKappa(Point2d_s p1, Point2d_s p2, Point2d_s p3);

/// 辅助函数，将vector中保存的(x, y)坐标转换为 eigen 的矩阵类型
Eigen::MatrixXd VectorToEigenMatrix(const std::vector<std::pair<double, double>> &input);

/**
* @brief 计算到黄色锥桶的距离
*/
std::pair<double, double> CalculateYellowDist(const std::vector<std::pair<double, double>> &yellow_points);

/**
 * @brief 插值算法
 *
 * @param points 待插值的曲线点集
 * @param interval_dis 插值间距
 * @param dis_threshold 距离阈值，两点间距离大于此值则进行插值
 */
std::vector<std::pair<double, double>> AverageInterpolation(const std::vector<std::pair<double, double>> &points,
															double interval_dis,
															double dis_threshold);

#endif  // reference_line_
