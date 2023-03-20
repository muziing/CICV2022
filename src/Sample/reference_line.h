#ifndef __reference_line__
#define __reference_line__
#pragma once

#include<iostream>
#include <vector>
#include <Eigen/Eigen>
#include <SensorBusDef.h>
#include <BasicsBusDef.h>
//#include <cmath>

struct Point3d_s {
  double x;
  double y;
  double z;
};

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
   * @brief save (x,y) according to color
   *
   * @param pLidar
   */
  void Shape(PanoSimSensorBus::Lidar_ObjList_G *pLidar);
  /**
   * @brief calc center point
   *
   */
  void CalcCenterPoint();
  /**
   * @brief sort center point
   *
   * @param pLidar
   * @param pEgo
   */
  void SortIndex();
  /* save center point (x,y) according to sort index
  */
  void CenterPoint();
  /**
   * @brief interpolation
   *
   * @param input enter_point_xy_sort
   * @param output enter_point_xy_final
   * @param interval_dis
   * @param distance max dis
   */
  void AverageInterpolation(Eigen::MatrixXd &input,
							std::vector<std::pair<double, double>> &output,
							double interval_dis,
							double distance);

  void CalcKTheta();

  std::vector<std::pair<double, double>> get_center_point_xy_sort() {
	return this->center_point_xy_sort;
  }

  std::vector<std::pair<double, double>> get_center_point_xy_final() {
	return this->center_point_xy_final;
  }
  std::vector<std::pair<double, double>> get_yellow_point_xy_final() {
	return this->yellow_xy;
  }
  void set_center_point_xy_final(std::vector<std::pair<double, double>> input) {
	this->center_point_xy_final = input;
  }
  double CalculateKappa(Point2d_s p1, Point2d_s p2, Point2d_s p3);

  void GetKappa(std::vector<std::pair<double, double>> center_point_xy_final);

  std::vector<RefPoint> GetRefMsg() {
	return this->RefMsg;
  }

  std::pair<double, double> CalculateYellowDist(const std::vector<std::pair<double, double>> &targetPath) {
	double x = 0, y = 0;
	for (auto point : targetPath) {
	  x += point.first;
	  y += point.second;
	}
	double yellow_dist = pow(x / targetPath.size(), 2)
		+ pow(y / targetPath.size(), 2);
	return std::make_pair(x / targetPath.size(), yellow_dist); // x_center dis
  }

 public:
  std::vector<std::pair<double, double>> center_point_xy;
  std::vector<int> match_point_index_set; // sort index in "CalcCenterPoint"
  std::vector<std::pair<double, double>> yellow_xy;
  std::vector<RefPoint> point; // 最终输出的点 x,y,kappa,theta

 private:
  std::vector<std::pair<double, double>> center_point_xy_sort; // center point after sort
  std::vector<std::pair<double, double>> center_point_xy_final; // center point after interpolation
  std::vector<std::pair<double, double>> in_xy; // (x,y)
  std::vector<std::pair<double, double>> out_xy;
  std::vector<RefPoint> RefMsg;
  int RefPointCounter{};
  std::vector<int> match_point_index_set_cen; // sort index  in "SortIndex"
  int inner = 0; // num of inner bucket
  int outer = 0;
};

#endif // !__reference_line__

