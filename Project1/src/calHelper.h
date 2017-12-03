/*
 * calHelper.h
 *
 *  Created on: Dec 1, 2017
 *      Author: magak
 */

#ifndef CALHELPER_H_
#define CALHELPER_H_

#include <vector>
#include <math.h>

using namespace std;

namespace calHelper
{
	// For converting back and forth between radians and degrees.
	constexpr double pi() { return M_PI; }

	double deg2rad(double x);
	double rad2deg(double x);
	double distance(double x1, double y1, double x2, double y2);
	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
}


#endif /* CALHELPER_H_ */
