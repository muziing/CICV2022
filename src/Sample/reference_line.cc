#include "reference_line.h"

void ReferenceLine::LoadTrafficConeData(PanoSimSensorBus::Lidar_ObjList_G *pLidar) {
  //std::cout << "---------" << std::endl;
  for (int i = 0; i < pLidar->header.width; ++i) {
	if (pLidar->items[i].shape == 2) {
	  // 左侧红色锥桶
	  this->out_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
	  // std::cout << out_xy[outer].first << "  " << out_xy[outer].second << std::endl;
	  this->outer++;
	} else if (pLidar->items[i].shape == 11) {
	  // 右侧蓝色锥桶
	  this->in_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
	  //std::cout << in_xy[inner].first << "  " << in_xy[inner].second << std::endl;
	  this->inner++;
	} else if (pLidar->items[i].shape == 13) {
	  // 起终点黄色锥桶
	  this->yellow_xy.emplace_back(pLidar->items[i].OBJ_S_X, pLidar->items[i].OBJ_S_Y);
	} else {
	  return;
	}
  }
}

void ReferenceLine::CalcCenterPoints() {
  std::vector<int> match_point_index_set;  // 存储内外侧锥桶间相匹配关系的索引
  for (auto &i : this->out_xy) {
	double min_dis = std::numeric_limits<double>::max();  // 以极大的数初始化最小距离，避免干扰
	int k = 0;

	// 匹配最近的内外侧锥桶对
	for (int j = 0; j < this->in_xy.size(); ++j) {
	  double dis = pow(i.first - this->in_xy[j].first, 2)
		  + pow(i.second - this->in_xy[j].second, 2);
	  // FIXME 这里的欧氏距离计算为什么不用 MatrixBase<Derived>::norm() 了？
	  if (dis < min_dis) {
		min_dis = dis;
		k = j;
	  }
	}
	match_point_index_set.push_back(k);
  }

  // 选出内外侧锥桶数中较少的一组
  size_t num_selected = this->in_xy.size() < this->out_xy.size() ? this->in_xy.size() : this->out_xy.size();

  for (size_t i = 0; i < num_selected; ++i) {
	this->center_point_xy.emplace_back((this->out_xy[i].first + this->in_xy[match_point_index_set[i]].first) / 2,
									   (this->out_xy[i].second + this->in_xy[match_point_index_set[i]].second)
										   / 2);
  }
}

void ReferenceLine::SortCenterPoints() {
  int index_cen = 0;
  std::vector<int> match_point_index_set_cen;
  std::vector<int> have_seen(this->center_point_xy.size());
  double min_dis = std::numeric_limits<double>::max();

  // 找到车后方最近的锥桶的索引
  for (int i = 0; i < this->center_point_xy.size(); ++i) {
	double dis = pow(this->center_point_xy[i].first, 2) + pow(this->center_point_xy[i].second, 2);
	if (dis < min_dis && center_point_xy[i].first < 0) {
	  min_dis = dis;
	  index_cen = i;
	}
  }
  have_seen[index_cen] = 1;  // 排好序的点标记为1
  match_point_index_set_cen.push_back(index_cen);

  int num = 0, j = 0, flag = -1;
  while (match_point_index_set_cen.size() < (this->center_point_xy.size() / 10)) {
	min_dis = std::numeric_limits<double>::max();
	for (int i = 0; i < this->center_point_xy.size(); ++i) {
	  double dis =
		  pow(this->center_point_xy[i].first - this->center_point_xy[match_point_index_set_cen[num]].first, 2) +
			  pow(this->center_point_xy[i].second - this->center_point_xy[match_point_index_set_cen[num]].second,
				  2);
	  if (dis < min_dis && have_seen[i] != 1 && (flag > 0 ? 1 : center_point_xy[i].first > 0)) {
		j = i;
		min_dis = dis;
	  }
	}
	flag++;
	match_point_index_set_cen.push_back(j);
	have_seen[j] = 1;
	num++;
  }

  // 按索引进行实际的排序
  for (int i = 0; i < this->center_point_xy.size() / 10; ++i) {
	this->center_points_xy_sorted.emplace_back(this->center_point_xy[match_point_index_set_cen[i]].first,
											   this->center_point_xy[match_point_index_set_cen[i]].second);
  }
}

void ReferenceLine::CalcKappaTheta() {
  std::vector<std::pair<double, double>> xy_set = this->GetCenterPointXyFinal();  // 得到中心点
  // 差分
  std::deque<std::pair<double, double>> dxy;  // (Δx, Δy)
  for (int i = 0; i < xy_set.size() - 1; ++i) {
	double dx = xy_set[i + 1].first - xy_set[i].first;
	double dy = xy_set[i + 1].second - xy_set[i].second;
	dxy.emplace_back(dx, dy);
  }
  std::deque<std::pair<double, double>> dxy_pre = dxy;
  std::deque<std::pair<double, double>> dxy_after = dxy;
  dxy_pre.emplace_front(dxy.front());  // 加上第一个数
  dxy_after.emplace_back(dxy.back());  // 加上最后一个数

  std::deque<std::pair<double, double>> dxy_final;
  for (int i = 0; i < xy_set.size(); ++i) {
	double dx = (dxy_pre[i].first + dxy_after[i].first) / 2;
	double dy = (dxy_pre[i].second + dxy_after[i].second) / 2;
	dxy_final.emplace_back(dx, dy);
  }

  // 计算 heading
  std::deque<double> front_theta;
  std::vector<double> ds_final;
  for (int i = 0; i < xy_set.size(); ++i) {
	double theta = atan2(dxy_final[i].second, dxy_final[i].first);
	front_theta.push_back(theta);

	// 计算每一段的弧长
	double ds = sqrt(pow(dxy_final[i].second, 2) + pow(dxy_final[i].first, 2));
	ds_final.push_back(ds);
  }

  std::deque<double> d_theta;
  for (int i = 0; i < xy_set.size() - 1; ++i) {
	// 计算 theta-diff
	double theta_diff = front_theta[i + 1] - front_theta[i];
	d_theta.push_back(theta_diff);
  }
  std::deque<double> d_theta_pre = d_theta;
  std::deque<double> d_theta_after = d_theta;
  d_theta_pre.push_front(d_theta.front());
  d_theta_after.push_back(d_theta.back());
  for (int i = 0; i < xy_set.size(); ++i) {
	double theta_final = (d_theta_pre[i] + d_theta_after[i]) / 2;
	this->path_points.push_back({xy_set[i].first, xy_set[i].second, sin(theta_final) / ds_final[i], front_theta[i]});
//	std::cout << "theta: " << front_theta[i] << std::endl;
//	if (i < 40) {
//		std::cout << sin(theta_final) / ds_final[i] << "\t";
//	}
  }
//  std::cout << std::endl;
}

void ReferenceLine::GetKappa(std::vector<std::pair<double, double>> final_center_point_xy) {
  Point2d_s p1{}, p2{}, a{}, b{}, c{};
  p1.x = final_center_point_xy[0].first;
  p1.y = final_center_point_xy[0].second;
  p2.x = final_center_point_xy[1].first;
  p2.y = final_center_point_xy[1].second;
  RefPoint r{};
  for (int i = 0; i < this->RefPointCounter - 2; ++i) {
	a.x = final_center_point_xy[i].first;
	a.y = final_center_point_xy[i].second;
	b.x = final_center_point_xy[i + 1].first;
	b.y = final_center_point_xy[i + 1].second;
	c.x = final_center_point_xy[i + 2].first;
	c.y = final_center_point_xy[i + 2].second;
	double k = CalculateKappa(a, b, c);
	r.x = a.x;
	r.y = a.y;
	r.kappa = k;
	RefMsg.emplace_back(r);
  }  // vector
  a.x = final_center_point_xy[RefPointCounter - 2].first;
  a.y = final_center_point_xy[RefPointCounter - 2].second;
  b.x = final_center_point_xy[RefPointCounter - 1].first;
  b.y = final_center_point_xy[RefPointCounter - 1].second;
  double k1 = CalculateKappa(a, b, p1);
  r.x = a.x;
  r.y = a.y;
  r.kappa = k1;
  RefMsg.emplace_back(r);
  double k2 = CalculateKappa(b, p1, p2);
  r.x = b.x;
  r.y = b.y;
  r.kappa = k2;
  RefMsg.emplace_back(r);
}

std::pair<double, double> CalculateYellowDist(const std::vector<std::pair<double,
																		  double>> &yellow_points) {
  double x = 0, y = 0;
  for (auto point : yellow_points) {
	x += point.first;
	y += point.second;
  }
  auto target_path_size = double(yellow_points.size());  // 避免向小范围隐式类型转换导致的编译器警告
  double yellow_dist = pow(x / target_path_size, 2)
	  + pow(y / target_path_size, 2);
  return std::make_pair(x / target_path_size, yellow_dist);  // x_center dis
}

double CalculateKappa(Point2d_s p1, Point2d_s p2, Point2d_s p3) {
  double a, b, c, sinA, cosA, r, k;
  a = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  b = sqrt((p2.x - p3.x) * (p2.x - p3.x) + (p2.y - p3.y) * (p2.y - p3.y));
  c = sqrt((p3.x - p1.x) * (p3.x - p1.x) + (p3.y - p1.y) * (p3.y - p1.y));
  cosA = (b * b + c * c - a * a) / (2 * b * c);
  sinA = sqrt(1 - cosA * cosA);
  r = 0.5 * a / sinA;
  k = 1 / r;
  return k;
}

Eigen::MatrixXd VectorToEigenMatrix(const std::vector<std::pair<double, double>> &input) {
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero((long long)input.size(), 2);
  for (long long i = 0; i < input.size(); ++i) {
	out(i, 0) = input[i].first;  // 第1列为x坐标
	out(i, 1) = input[i].second;  // 第2列为y坐标
  }
  return out;
}

std::vector<std::pair<double, double>> AverageInterpolation(const std::vector<std::pair<double, double>> &points,
															double interval_dis,
															double dis_threshold) {
  Eigen::MatrixXd input = VectorToEigenMatrix(points);  // 先转换为矩阵，方便计算
  std::vector<std::pair<double, double>> output;  // 最终输出时再转换回vector
  // 1.定义一个容器，类型为Point2d_s,即（x, y）
  std::vector<Point2d_s> vec_2d;
  Point2d_s p{};
  // 2.遍历
  for (long long i = 0; i < input.rows() - 1; ++i) {
	double dis = (input.row(i + 1) - input.row(i)).norm();  // 该点到下一点的距离
	// 对间距太长的两点间进行插点
	if (dis >= dis_threshold) {
	  // 计算(x,y)两点的距离
	  double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
		  (input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
	  // 计算角度
	  double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
	  double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
	  // 两点之间要插值的插值点的数量
	  int num = int(dis / interval_dis);
	  // 插入点
	  for (int j = 0; j < num; j++) {
		// i=0,j=0
		p.x = input(i, 0) + j * interval_dis * cos_a;
		p.y = input(i, 1) + j * interval_dis * sin_a;
		vec_2d.push_back(p);
	  }
	} else {
	  // 有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
	  p.x = input(i, 0);
	  p.y = input(i, 1);
	  vec_2d.push_back(p);
	}
  }
  // 3.补回最后一个点
  p.x = input(input.rows() - 1, 0);
  p.y = input(input.rows() - 1, 1);
  vec_2d.push_back(p);
  // 4.output
  for (auto &it : vec_2d) {
	output.emplace_back(it.x, it.y);
  }
  return output;
}
