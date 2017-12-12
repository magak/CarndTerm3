/*
 * Predictor.cpp
 *
 *  Created on: Dec 7, 2017
 *      Author: magak
 */

#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <iostream>
#include <random>

#include "Predictor.h"
#include "Road.h"

using namespace std;

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
	_lastSensorFusion.clear();

	for(vector<vector<double>>::iterator it = sensor_fusion.begin(); it != sensor_fusion.end(); ++it)
	{
		int id = (*it)[0];
		VEHICLE vh;
		vh.id = id;

		double vx = (*it)[3];
		double vy = (*it)[4];

		vh.v = sqrt(vx*vx+vy*vy);

		vh.s = (*it)[5];
		vh.d = (*it)[6];

		for(int lane = 0; lane < _road.LanesAvailable; lane++)
		{
			if(vh.d < (_road.LaneWidth*(lane+1)) && vh.d > (_road.LaneWidth*lane))
			{
				vh.lanes.push_back(lane);
			}


			if(  abs( ((double)lane+0.5)*_road.LaneWidth - (double)vh.d ) < 0.62*_road.LaneWidth )
			{
				vh.lanes.push_back(lane);
			}

		}

		_lastSensorFusion[id] = vh;
	}
}

map<int, VEHICLE> Predictor::GetPredictions(double timeHorizon)
{
	map<int, VEHICLE> result;
	for(map<int, VEHICLE>::iterator it = _lastSensorFusion.begin(); it != _lastSensorFusion.end(); ++it)
	{
		VEHICLE veh;
		veh.id = it->first;
		veh.v = it->second.v;
		veh.d = it->second.d;
		veh.s = it->second.s+timeHorizon*it->second.v;
		veh.lanes = it->second.lanes;
		result[it->first] = veh;
	}

	return result;
}
