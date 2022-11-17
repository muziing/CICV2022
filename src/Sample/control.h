#ifndef _CONTROL_
#define _CONTROL_

#pragma once
#include "reference_line.h"

/**
 * @brief control class, lateral && longitudinal
 * 
 */

class control
{
public:
	control()=default;
	virtual ~control()=default;

	// virtual function, base, lqr_control and pure_suit is inherited
	virtual double calculateCmd(const std::vector<RefPoint> &targetPath, PanoSimBasicsBus::Ego *pEgo) = 0;

	// virtual int findTrajref(const std::vector<std::pair<double, double>> &targetPath, PanoSimBasicsBus::Ego *pEgo) = 0;

public:
	// calc forwardindex
	static size_t calc_forwardIndex(const std::vector<std::pair<double, double>>& targetPath, PanoSimBasicsBus::Ego* pEgo) {
		// Nearest point index after sort is 0
		size_t index = 0;

		size_t forwardIndex = 0;
		double minProgDist = 1.5;
		double progTime = 0.5;
		double mainVehicleSpeed = pEgo->speed;
		double progDist = mainVehicleSpeed * progTime > minProgDist ? mainVehicleSpeed * progTime : minProgDist;

		if (calculateKappa(targetPath, 1) > 0.3) {
			progDist = 0.5;
			//std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
		}
		for (; index < targetPath.size(); ++index) {
			forwardIndex = index;
			double distance = sqrtf((double)pow(targetPath[index].first, 2) +
				pow((double)targetPath[index].second, 2));
			if (distance >= progDist) {
				return forwardIndex;
			}
		}
		return 0;
	}

	static double calculateSteering(const std::vector<std::pair<double, double>>& targetPath, PanoSimBasicsBus::Ego* pEgo, size_t forwardIndex) {
		
		// std::cout << "forwardIndex: " << forwardIndex << std::endl;
		double deltaAlfa = atan2(targetPath[forwardIndex].second,
			targetPath[forwardIndex].first);// alfa
		double ld = sqrt(pow(targetPath[forwardIndex].second, 2) +
			pow(targetPath[forwardIndex].first, 2)); // distance 
		double steer = atan2(2. * (1.55) * sin(deltaAlfa), ld) * 180 * 3.67 / (1 * M_PI);
		
		if (steer > 120) {
			steer = 120;
		}
		else if (steer < -120) {
			steer = -120;
		}
		return steer;
	}

	static double calculateThrottleBreak(const std::vector<std::pair<double, double>>& targetPath, PanoSimBasicsBus::Ego* pEgo, size_t forwardIndex) {

		auto nearKappa = calculateKappa(targetPath, 0);
		auto farKappa = calculateKappa(targetPath, forwardIndex);
		auto lastKappa = calculateKappa(targetPath, targetPath.size() - 11);
		double this_kappa = 0.01;
		if (nearKappa > farKappa) {
			this_kappa = nearKappa;
		}
		else {
			this_kappa = farKappa;
		}
		if (lastKappa > 0.20) this_kappa = lastKappa * 0.55;
		this_kappa = this_kappa < 0.012 ? 0.012 : this_kappa;

		auto max_v = sqrt( 2.2 / this_kappa);
		// std::cout << "longtitude forwardIndex: " << forwardIndex << std::endl;
		// std::cout << "nearKappa : " << nearKappa << "\t farKappa : " << farKappa << "\t lastKappa :" << lastKappa << std::endl;
		// std::cout << "max_v is :" << max_v  << "\tand pEgo->speed is : " << pEgo->speed << std::endl;
		// std::cout << "targetPath.size() is :" << targetPath.size() << std::endl;
		// std::cout << "this_kappa is :" << this_kappa << std::endl;
		// std::cout << "-----------------" << std::endl;
		return PID_Control(max_v > 0 ? 2.0 : max_v, pEgo->speed);
	}

	static double PID_Control(double value_target, double value_now) {
		double dt = 0.01;
		double kp = 0.30;
		double ki = 0.1;
		double kd = 0.01;

		double value_p = (value_target - value_now) / value_target;
		double value_i = 0;
		value_i += (value_target - value_now) * dt / value_target;
		double value_last = 0;
		double value_d = (value_now - value_last) / dt;
		
		double control_value = kp * value_p + ki * value_i + kd * value_d;
		// std::cout << "control_value is : " << control_value << std::endl;
		if (control_value > 1) control_value = 1;
		if (control_value < -1) control_value = -1;
		// std::cout << "control_value after limit is : " << control_value << std::endl;
		value_last = value_now;
		return control_value;

	}

	static double calculateKappa(const std::vector<std::pair<double, double>>& targetPath, int idx) {
		Point2d_s p1, p2, p3;
		if (idx + 10 < targetPath.size()) {
			p1.x = targetPath[idx].first;
			p1.y = targetPath[idx].second;
			p2.x = targetPath[idx + 5].first;
			p2.y = targetPath[idx + 5].second;
			p3.x = targetPath[idx + 10].first;
			p3.y = targetPath[idx + 10].second;
		}
		else {
			int num = idx + 10 - targetPath.size();
			p1.x = targetPath[idx].first;
			p1.y = targetPath[idx].second;
			p2.x = targetPath[num].first;
			p2.y = targetPath[num].second;
			p3.x = targetPath[num + 5].first;
			p3.y = targetPath[num + 5].second;
		}
		
		
		double a, b, c, sinA, cosA, r, k;

		a = sqrt(abs((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)));
		b = sqrt(abs((p2.x - p3.x) * (p2.x - p3.x) + (p2.y - p3.y) * (p2.y - p3.y)));
		c = sqrt(abs((p3.x - p1.x) * (p3.x - p1.x) + (p3.y - p1.y) * (p3.y - p1.y)));

		cosA = (b * b + c * c - a * a) / (2 * b * c > 0.01 ? 2 * b * c : 0.01);
		sinA = sqrt(1 - cosA * cosA);
		r = 0.5 * a / (sinA > 0.005 ? sinA : 0.005);
		k = 1 / (r > 0.01 ? r : 0.01);
		return k;

	}

public:
	/*static size_t point_index;
	static double value_i;
	static double value_last;*/

};





#endif // !_CONTROL_



