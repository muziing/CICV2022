#include <PanoSimApi.h>
#include <string>
#include <string_view>
#include <fstream>

#include "control.h"
#include "lqr_control.h"
#include "pure_puresuit_control.h"

using std::string;
using std::string_view;
using std::vector;
using std::cout;
using std::endl;
using std::move;

using PanoSimSensorBus::LIDAR_OBJLIST_G_FORMAT;
using PanoSimSensorBus::Lidar_ObjList_G;
using PanoSimBasicsBus::EGO_CONTROL_FORMAT;
using PanoSimBasicsBus::EgoControl;
using PanoSimBasicsBus::EGO_FORMAT;
using PanoSimBasicsBus::Ego;
using PanoSimBasicsBus::EGO_EXTRA_FORMAT;
using PanoSimBasicsBus::EgoExtra;

struct GlobalData {
  BusAccessor *lidar;
  BusAccessor *ego_control, *ego;
  int times = 0;  // 经过起终点的次数（由此推出当前圈数）
  bool flg = false;
  std::shared_ptr<Control> control_base;
};

void PrintParameters(UserData *userData);
void SplitString(const string_view strSrc, const string_view strSeparator, vector<string> &vctSplitResult);

/// 仿真启动时运行一次的初始化函数
void ModelStart(UserData *userData) {
  PrintParameters(userData);

  auto pGlobal = new GlobalData;
  pGlobal->lidar = new BusAccessor(userData->busId, "Lidar_ObjList_G.0", LIDAR_OBJLIST_G_FORMAT);
  pGlobal->ego_control = new BusAccessor(userData->busId, "ego_control", EGO_CONTROL_FORMAT);
  pGlobal->ego = new BusAccessor(userData->busId, "ego", EGO_FORMAT);
  userData->state = pGlobal;

  // 横向控制模式 0:LQR  1:纯跟踪
  int control_mode = 0;
  switch (control_mode) {
	case 0:cout << "LQR init!!!";
	  pGlobal->control_base = std::make_shared<LqrControl>(0.1, 0.06, 0.0);
	  break;
	case 1:cout << "Pure-pursuit init!!!";
	  pGlobal->control_base = std::make_shared<PurePursuit>(0.1, 0.06, 0.0);
	  break;
	default:break;
  }
}

/// 回调函数（仿真中的主要）
void ModelOutput(UserData *userData) {

  if (userData != nullptr) {
	auto pGlobal = static_cast<GlobalData *>(userData->state);
	if (pGlobal != nullptr) {
	  Lidar_ObjList_G *pLidar = nullptr;
	  Ego *pEgo = nullptr;

	  ReferenceLine reference_line;  // 创建一个 reference_line 实例

	  if (pGlobal->lidar != nullptr && pGlobal->ego != nullptr) {
		pLidar = static_cast<Lidar_ObjList_G *>(pGlobal->lidar->GetHeader());
		pEgo = static_cast<Ego *>(pGlobal->ego->GetHeader());

		// 路径规划开始
		// TODO 重构此段代码，在ReferenceLine类中提供直接输出最终结果的成员函数
		reference_line.LoadTrafficConeData(pLidar);
		reference_line.CalcCenterPoints();
		reference_line.SortCenterPoints();

		// 插值
		if (!reference_line.GetCenterPointXySort().empty()) {
		  std::vector<std::pair<double, double>>
			  output = AverageInterpolation(reference_line.GetCenterPointXySort(), 0.2, 0.6);
		  reference_line.SetCenterPointXyFinal(output);
		  //std::cout << "+++++++++++++++++++++" << std::endl;
		  /*for (auto line : output) {
			  cout << line.first << "   " << line.second << endl;
		  }*/
		}
		reference_line.CalcKappaTheta();  // struct RefPoint
		// 路径规划结束
	  }

	  // Control class
	  EgoControl *pEgoCtrl = nullptr;
	  if (pGlobal->ego_control != nullptr) {
		double steer = 0.0;
		pEgoCtrl = static_cast<EgoControl *>(pGlobal->ego_control->GetHeader());

		// FIXME 可能存在的问题：如果不满足上一段的条件语句，未进行路径规划，则reference_line.path_points为空，会引起后续问题吧
		// 所以上一段的条件语句是干啥用的？
		std::vector<RefPoint> target_path_points = reference_line.path_points;  // 最终的期望路径

		steer = pGlobal->control_base->CalculateCmd(target_path_points, pLidar, pEgo);
		int forwardIndex = pGlobal->control_base->CalcForwardIndex(target_path_points, pEgo);
		// cout << "sample steer: " << steer << endl;
		double throttle = pGlobal->control_base->CalculateThrottleBreak(target_path_points, pEgo, forwardIndex);
		std::pair<double, double>
			yellow_dist = CalculateYellowDist(reference_line.GetYellowPoints());

		pEgoCtrl->time = userData->time;
		pEgoCtrl->valid = 1;
		if (pGlobal->times < 4) {
		  if (throttle > 0) {
			pEgoCtrl->throttle = throttle;
			pEgoCtrl->brake = 0;
		  } else {
			pEgoCtrl->throttle = 0;
			pEgoCtrl->brake = -throttle;
		  }
		  pEgoCtrl->steer = steer;
		  pEgoCtrl->mode = 1;
		  pEgoCtrl->gear = 1;

		  // 停车控制 begin
		  if (yellow_dist.second < 1 && yellow_dist.first > 0 && !pGlobal->flg) {
			pGlobal->flg = true;
		  } else if (yellow_dist.second < 1 && yellow_dist.first < 0 && pGlobal->flg) {
			pGlobal->flg = false;
			pGlobal->times++;
		  }
		} else {
		  // 已经是第四次经过黄色锥桶
		  pEgoCtrl->throttle = 0;
		  pEgoCtrl->brake = 1;
		}
		// 停车控制 end
	  }
	}
  }
}

/// 仿真结束时运行一次的收尾清理函数
void ModelTerminate(UserData *userData) {
  if (userData->state != nullptr) {
	auto pGlobal = static_cast<GlobalData *>(userData->state);
	if (pGlobal != nullptr) {
	  if (pGlobal->lidar != nullptr) {
		delete pGlobal->lidar;
		pGlobal->lidar = nullptr;
	  }
	  if (pGlobal->ego_control != nullptr) {
		delete pGlobal->ego_control;
		pGlobal->ego_control = nullptr;
	  }
	  delete pGlobal;
	  userData->state = nullptr;
	}
  }
}

/// 辅助函数，调试用，打印输出 parameters
void PrintParameters(UserData *userData) {
  for (const auto &pairItem : userData->parameters) {
	cout << pairItem.first << ":" << pairItem.second << endl;
  }

  cout << userData->busId << endl;
  cout << userData->name << endl;

  const char *key_parameter = "Parameters";
  auto findParam = userData->parameters.find(key_parameter);
  if (findParam != userData->parameters.end()) {
	vector<string> vctParameter;
	constexpr
	string_view parameter_separator = ",";
	SplitString(findParam->second, parameter_separator, vctParameter);
	for (const auto &strParameter : vctParameter) {
	  cout << strParameter << endl;
	}
  }
}

/// 辅助函数，由指定的标识符分割字符串
void SplitString(const string_view strSrc, const string_view strSeparator, vector<string> &vctSplitResult) {
  vctSplitResult.clear();
  string::size_type nBegin = 0;
  string::size_type nEnd = strSrc.find(strSeparator);
  while (string::npos != nEnd) {
	vctSplitResult.emplace_back(strSrc.substr(nBegin, nEnd - nBegin));
	nBegin = nEnd + strSeparator.size();
	nEnd = strSrc.find(strSeparator, nBegin);
  }
  if (nBegin != strSrc.length()) {
	vctSplitResult.emplace_back(strSrc.substr(nBegin));
  }
}
