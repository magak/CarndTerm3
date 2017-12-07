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

#include "Road.h"

using namespace std;

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

private:
	vector<vector<double>> _lastSensorFusion;
	ROAD_CONFIGURATION _road;
};

#endif /* PREDICTOR_H_ */
