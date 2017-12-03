/*
 * EgoVehicle.cpp
 *
 *  Created on: Dec 1, 2017
 *      Author: magak
 */

#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <iostream>
#include <random>
#include "EgoVehicle.h"
#include "json.hpp"
#include "spline.h"
#include "calHelper.h"

using namespace std;
using namespace calHelper;

/**
 * constructor
 */
EgoVehicle::EgoVehicle(int currentLane, double laneWidth,
		vector<double> map_waypoints_x,
		vector<double> map_waypoints_y,
		vector<double> map_waypoints_s,
		vector<double> map_waypoints_dx,
		vector<double> map_waypoints_dy,
		EgoVehicleState state) {
	_currentLane = currentLane;
	_targetLane = currentLane;
	_laneWidth = laneWidth;
	_map_waypoints_x = map_waypoints_x;
	_map_waypoints_y = map_waypoints_y;
	_map_waypoints_s = map_waypoints_s;
	_map_waypoints_dx = map_waypoints_dx;
	_map_waypoints_dy = map_waypoints_dy;
}

void EgoVehicle::ProcessInputData(
		double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
		nlohmann::basic_json<>& previous_path_x_orig, nlohmann::basic_json<>& previous_path_y_orig,
		double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion,
		vector<double> &next_x_vals, vector<double> &next_y_vals
		){
	_car_x = car_x;
	_car_y = car_y;
	_car_s = car_s;
	_car_d = car_d;
	_car_yaw = car_yaw;
	_car_speed = car_speed;
	_end_path_s = end_path_s;
	_end_path_d = end_path_d;
	_sensor_fusion = sensor_fusion;

  	int limit_prev_size = previous_path_x_orig.size();
  	vector<double> previous_path_x;
  	vector<double> previous_path_y;

  	/*
  	if(limit_prev_size > 15)
  		limit_prev_size = 15;
  	*/

  	for(int i = 0; i < limit_prev_size; i++)
  	{
  		previous_path_x.push_back(previous_path_x_orig[i]);
  		previous_path_y.push_back(previous_path_y_orig[i]);
  	}

	_previous_path_x = previous_path_x;
	_previous_path_y = previous_path_y;

	auto keepLaneTraj = GenerateKeepLaneTrajectory();
	next_x_vals = keepLaneTraj.next_x_vals;
	next_y_vals = keepLaneTraj.next_y_vals;
	_targetLane = keepLaneTraj.targetLane;
	_targetVel = keepLaneTraj.targetVel;

	//int size = previous_path_x.size();
	//cout << size << endl;
}

TRAJECTORY EgoVehicle::GenerateKeepLaneTrajectory()
{
	TRAJECTORY result;
	result.targetLane = this->_targetLane;
	result.targetVel = this->_targetVel;

	int prev_size = _previous_path_x.size();

	if(prev_size > 0)
	{
		_car_s = _end_path_s;
	}

	bool too_close = false;

	for(int i = 0; i < _sensor_fusion.size(); i++)
	{
		float d = _sensor_fusion[i][6];
	    if(d < (_laneWidth*(result.targetLane+1)) && d > (_laneWidth*result.targetLane))
	    {
	    	double vx = _sensor_fusion[i][3];
	        double vy = _sensor_fusion[i][4];
	        double check_speed = sqrt(vx*vx+vy*vy);
	        double check_car_s = _sensor_fusion[i][5];

	        check_car_s+=((double)prev_size*delta_t*check_speed);
	        if((check_car_s > _car_s) && ( (check_car_s-_car_s) < planXHorizon) )
	        {
	        	too_close = true;
	        	if(result.targetLane > 0)
	        	{
	        		result.targetLane = 0;
	        	}
	        }
	    }
	}

	if(too_close)
	{
		result.targetVel -= _veloctyChangeRate;
	}
	else if(result.targetVel < _maxVelocity)
	{
		result.targetVel += _veloctyChangeRate;
	}

	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = _car_x;
	double ref_y = _car_y;
	double ref_yaw = calHelper::deg2rad(_car_yaw);

	if(prev_size < 2)
	{
		double prev_car_x = _car_x - cos(_car_yaw);
		double prev_car_y = _car_y - sin(_car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(_car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(_car_y);
	}
	else
	{
		ref_x = _previous_path_x[prev_size-1];
		ref_y = _previous_path_y[prev_size-1];

		double ref_x_prev = _previous_path_x[prev_size-2];
		double ref_y_prev = _previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	vector<double> next_wp0 = calHelper::getXY(_car_s+planXHorizon,(_laneWidth/2+_laneWidth*_targetLane),_map_waypoints_s, _map_waypoints_x, _map_waypoints_y);
	vector<double> next_wp1 = calHelper::getXY(_car_s+planXHorizon*2,(_laneWidth/2+_laneWidth*_targetLane),_map_waypoints_s, _map_waypoints_x, _map_waypoints_y);
	vector<double> next_wp2 = calHelper::getXY(_car_s+planXHorizon*3,(_laneWidth/2+_laneWidth*_targetLane),_map_waypoints_s, _map_waypoints_x, _map_waypoints_y);

	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for(int i = 0; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i]-ref_x;
		double shift_y = ptsy[i]-ref_y;

		ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
		ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	}

	tk::spline s;

	s.set_points(ptsx, ptsy);

	for(int i = 0; i < _previous_path_x.size(); i++)
	{
		result.next_x_vals.push_back(_previous_path_x[i]);
		result.next_y_vals.push_back(_previous_path_y[i]);
	}

	double prevDist = 0;
	if(_previous_path_x.size() > 2)
	{
		prevDist = calHelper::distance(result.next_x_vals[_previous_path_x.size()-1],
				result.next_y_vals[_previous_path_x.size()-1],
				result.next_x_vals[_previous_path_x.size()-2],
				result.next_y_vals[_previous_path_x.size()-2]);
	}

	double target_x = planXHorizon;
	double target_y = s(target_x);
	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

	double x_add_on = 0;

	for(int i = 1; i <= pathPointCount-_previous_path_x.size(); i++){

		double N = (target_dist/(delta_t*result.targetVel /speedMPHRatio));
		double x_point = x_add_on+(target_x)/N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// rotate back
		x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

		x_point += ref_x;
		y_point += ref_y;

		result.next_x_vals.push_back(x_point);
		result.next_y_vals.push_back(y_point);
	}

	result.found = true;

	return result;
}
