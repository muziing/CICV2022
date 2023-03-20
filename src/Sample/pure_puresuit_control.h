#ifndef __PURE_PURSUIT__
#define __PURE_PURSUIT__

#pragma once
#include "control.h"

class PurePursuit : public Control {
 public:
  PurePursuit(const double kp, const double ki, const double kd);
  ~PurePursuit() = default;

  double CalculateCmd(const std::vector<RefPoint> &targetPath, PanoSimSensorBus::Lidar_ObjList_G *pLidar,
					  PanoSimBasicsBus::Ego *pEgo) override;
};

#endif // !__PURE_PURSUIT__

