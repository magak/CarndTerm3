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

/*
 * represents the vehicle parameters on the road
 */
struct VEHICLE
{
	int id;
	double s; // s param of frenet coords
	double d; // d param of frenet coords
	double v; // speed
	vector<int> lanes;
};

/*
 * Predictor object
 */
class Predictor
{
public:
	/*
	 * Constructors
	 */
	Predictor(ROAD_CONFIGURATION road);
	Predictor();

	/*
	 * Feeding the data from sensor fusion
	 */
	void ProcessData(vector<vector<double>> sensor_fusion);

	/*
	 * Returns the predictions of vehicles on the road given the time horizon specified
	 */
	map<int, VEHICLE> GetPredictions(double timeHorizon);

private:
	/*
	 * last sensor fusion data processed
	 */
	map<int, VEHICLE> _lastSensorFusion;

	/*
	 * road parameters
	 */
	ROAD_CONFIGURATION _road;
};

#endif /* PREDICTOR_H_ */
