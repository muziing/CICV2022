#ifndef _CONTROL_
#define _CONTROL_

#pragma once
#include "reference_line.h"

/**
 * @brief Control class, lateral && longitudinal
 * 
 */
class Control {
 public:
  Control(double kp, double ki, double kd);
  virtual ~Control() = default;

  // virtual function, base, LqrControl and pure_suit is inherited
  virtual double CalculateCmd(const std::vector<RefPoint> &targetPath, PanoSimSensorBus::Lidar_ObjList_G *pLidar,
							  PanoSimBasicsBus::Ego *pEgo) = 0;

 public:
  // calc forward-index
  int CalcForwardIndex(const std::vector<RefPoint> &targetPath, PanoSimBasicsBus::Ego *pEgo);

  double CalculateThrottleBreak(const std::vector<RefPoint> &targetPath,
								PanoSimBasicsBus::Ego *pEgo,
								size_t forwardIndex);

  double PidControl(double value_target, double value_now);

  double CalculateKappa(const std::vector<RefPoint> &targetPath, int idx);

  void Reset();

 protected:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double error_sub_ = 0.0;
  double previous_error_ = 0.0;
  double integral_ = 0.0;
  double differential_ = 0.0;
  bool first_init_ = false;
};

#endif // !_CONTROL_



