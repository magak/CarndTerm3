/*
 * Predictor.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: magak
 */

#include <math.h>
#include <vector>

#include "Predictor.h"
#include "Road.h"

Predictor::Predictor(ROAD_CONFIGURATION road)
{
	_road = road;
}

Predictor::Predictor()
	:Predictor({4.0, 3})
{

}

void Predictor::ProcessData(vector<vector<double>> sensor_fusion)
{
	_lastSensorFusion = sensor_fusion;
}
