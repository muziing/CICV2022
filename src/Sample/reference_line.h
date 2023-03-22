#ifndef reference_line_
#define reference_line_
#pragma once

#include<iostream>
#include <vector>
#include <Eigen/Eigen>
#include <SensorBusDef.h>
#include <BasicsBusDef.h>
//#include <cmath>


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
   * @brief 按颜色分别存储所有锥桶坐标
   *
   * @param pLidar
   */
  void Shape(PanoSimSensorBus::Lidar_ObjList_G *pLidar);

  /**
   * @brief 计算并存储所有中心点（未排序）
   */
  void CalcCenterPoint();

  /**
   * @brief 对所有中心点按路径顺序排序，获得索引
   */
  void SortIndex();

  /**
   * @brief 保存对中心点进行了排序的中心点坐标
  */
  void CenterPoint();

  /**
   * @brief 插值算法
   *
   * @param input enter_point_xy_sort
   * @param output enter_point_xy_final
   * @param interval_dis 插值间距
   * @param distance_threshold 距离阈值，两点间距离大于此值则进行插值
   */
  static std::vector<std::pair<double, double>> AverageInterpolation(const Eigen::MatrixXd &input,
																	 double interval_dis,
																	 double distance_threshold);

  /// 计算 Kappa theta,并保存至 path_points 中
  void CalcKappaTheta();

  std::vector<std::pair<double, double>> GetCenterPointXySort() {
	return this->center_point_xy_sort;
  }

  std::vector<std::pair<double, double>> GetCenterPointXyFinal() {
	return this->center_point_xy_final;
  }
  std::vector<std::pair<double, double>> GetYellowPointXyFinal() {
	return this->yellow_xy;
  }
  void SetCenterPointXyFinal(const std::vector<std::pair<double, double>> &center_points) {
	this->center_point_xy_final = center_points;  // FIXME 在进行插值计算之后自动设置，而不是显式调用此函数
  }
  static double CalculateKappa(Point2d_s p1, Point2d_s p2, Point2d_s p3);  // FIXME 理清和GetKappa的关系

  void GetKappa(std::vector<std::pair<double, double>> final_center_point_xy);

  /**
  * @brief 计算到黄色锥桶的距离
  */
  static std::pair<double, double> CalculateYellowDist(const std::vector<std::pair<double, double>> &target_path);

 public:
  std::vector<RefPoint> path_points;  // 最终输出的点 x, y, kappa, theta

 private:
  int RefPointCounter{};
  size_t inner = 0;  // 内侧锥桶个数
  size_t outer = 0;  // 外侧锥桶个数
  std::vector<int> match_point_index_set;  // 存储内外侧锥桶间相匹配关系的索引
  std::vector<std::pair<double, double>> center_point_xy;  // 所有中心点的坐标
  std::vector<int> match_point_index_set_cen;  // sort index  in "SortIndex"
  std::vector<std::pair<double, double>> center_point_xy_sort;  // 保存排序后的所有中心点坐标
  std::vector<std::pair<double, double>> center_point_xy_final;  // 插值后的所有中心点坐标
  std::vector<std::pair<double, double>> in_xy;  // 存储所有右侧蓝色锥桶的(x, y)坐标
  std::vector<std::pair<double, double>> out_xy;  // 存储所有左侧红色锥桶的(x, y)坐标
  std::vector<std::pair<double, double>> yellow_xy;  // 存储所有起终点黄色锥桶的(x, y)坐标
  std::vector<RefPoint> RefMsg; // FIXME 貌似没用用上，干什么用的？
};

#endif  // reference_line_

