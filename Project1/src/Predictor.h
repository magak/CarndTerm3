/*
 * Predictor.h
 *
 *  Created on: Dec 6, 2017
 *      Author: magak
 */

#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include <math.h>
#include <vector>
#include <map>

#include "Road.h"

using namespace std;

struct VEHICLE
{
	int id;
	double s;
	double d;
	double v;
};

class Predictor
{
public:
	/*
	 * Constructor
	 */
	Predictor(ROAD_CONFIGURATION road);
	Predictor();

	/*
	 * Feeding data
	 */
	void ProcessData(vector<vector<double>> sensor_fusion);

	map<int, VEHICLE> GetPredictions(double timeHorizon);

	/*
	VEHICLE getAheadVehicle(int lane, double currentS);
	VEHICLE getBehindVehicle(int lane, double currentS);
	*/

private:
	map<int, VEHICLE> _lastSensorFusion;
	ROAD_CONFIGURATION _road;
};

#endif /* PREDICTOR_H_ */
