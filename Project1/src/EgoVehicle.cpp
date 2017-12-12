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
#include "Road.h"
#include "Predictor.h"

using namespace std;
using namespace calHelper;

/**
 * constructor
 */
EgoVehicle::EgoVehicle(int currentLane, ROAD_CONFIGURATION road,
		vector<double> map_waypoints_x,
		vector<double> map_waypoints_y,
		vector<double> map_waypoints_s,
		vector<double> map_waypoints_dx,
		vector<double> map_waypoints_dy,
		EgoVehicleState state) {
	_currentLane = currentLane;
	_targetLane = currentLane;
	_map_waypoints_x = map_waypoints_x;
	_map_waypoints_y = map_waypoints_y;
	_map_waypoints_s = map_waypoints_s;
	_map_waypoints_dx = map_waypoints_dx;
	_map_waypoints_dy = map_waypoints_dy;
	_currentState = state;
	_road = road;

	_predictor = Predictor(_road);
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

	_predictor.ProcessData(_sensor_fusion);

  	int limit_prev_size = previous_path_x_orig.size();
  	_prev_size = limit_prev_size;

  	if(limit_prev_size > 5)
  		limit_prev_size = 5;

  	vector<double> previous_path_x;
  	vector<double> previous_path_y;

  	for(int i = 0; i < limit_prev_size; i++)
  	{
  		previous_path_x.push_back(previous_path_x_orig[i]);
  		previous_path_y.push_back(previous_path_y_orig[i]);
  	}

	_previous_path_x = previous_path_x;
	_previous_path_y = previous_path_y;

	if(limit_prev_size > 0)
	{
		_car_s = _end_path_s;
	}

	auto bestTraj = GetBestTrajectory();

	next_x_vals = bestTraj.next_x_vals;
	next_y_vals = bestTraj.next_y_vals;
	_targetLane = bestTraj.targetLane;
	_targetVel = bestTraj.targetVel;
	_vcte = bestTraj.vcte;
	_currentState = bestTraj.state;

	double ddiff = abs(_car_d - ((double)_targetLane+0.5)*_road.LaneWidth);
	//cout << "ddiff=" << ddiff << endl;
	//cout << statesStr[(int)_currentState] << endl;
}

TRAJECTORY EgoVehicle::GetBestTrajectory()
{
	vector<TRAJECTORY> trajectories;
	double cost;
	vector<double> costs;
	vector<EgoVehicleState> states = getSuccessorStates();

	map<int, VEHICLE> predictions = _predictor.GetPredictions((double)_prev_size*delta_t);

	VEHICLE vhAhead0 = getVehicleAhead(0, _car_s, predictions);
	VEHICLE vhAhead1 = getVehicleAhead(1, _car_s, predictions);
	//VEHICLE vhAhead2 = getVehicleAhead(2, _car_s, predictions);

	//cout << vhAhead0.id << " " << vhAhead1.id << " " << vhAhead2.id << endl;

	//TRAJECTORY traj = GenerateKeepLaneTrajectory(predictions);
	//traj.state = EgoVehicleState::KeepLane;
	//return traj;

	/*
	double s1 = calcSpaceAheadAvailable(0, predictions);
	double s2 = calcSpaceAheadAvailable(1, predictions);
	double s3 = calcSpaceAheadAvailable(2, predictions);

	double m1 = calcMaxSpaceAheadAvailable(0, predictions);
	double m2 = calcMaxSpaceAheadAvailable(1, predictions);
	double m3 = calcMaxSpaceAheadAvailable(2, predictions);
	cout << "s1=" << s1 << "	s2=" << s2 << "	s3=" << s3 << "____" << "	m1=" << m1 << "	m2=" << m2 << "	m3=" << m3 << endl;
	*/

	string statesStr[] = { "KeepLane", "LaneChangeLeft", "LaneChangeRight", "LaneChanging"};
	for(vector<EgoVehicleState>::iterator it = states.begin(); it != states.end(); ++it)
	{
		TRAJECTORY trajectory = GenerateTrajectoryForState(*it, predictions);
		trajectory.state = *it;

		if(trajectory.found)
		{
			trajectory.maxSpaceAheadAvailable = calcMaxSpaceAheadAvailable(trajectory.targetLane, predictions);
			trajectory.spaceAheadAvailable = calcSpaceAheadAvailable(trajectory.targetLane, predictions);
			cost = calcCost(trajectory);
			costs.push_back(cost);
			trajectories.push_back(trajectory);

			/*
			 * some diagnostics
			double v1 = _velocityWeight*(_maxVelocity-trajectory.targetVel);
			double v2 = _maxSpaceAheadWeight*(farPlanXHorizon-trajectory.maxSpaceAheadAvailable);
			double v3 = _spaceAheadWeight*(farPlanXHorizon-trajectory.spaceAheadAvailable);
			double v4 = _stateWeight*((double)trajectory.state);
			double vcte = trajectory.vcte;

			cout << statesStr[(int)(*it)] << "=" << cost;
			cout << "(" << "vcte=" << vcte << ", vel=" << v1 << ", maxSpace=" << v2 << ", space=" << v3 << ", state=" << v4 << ")" << "		";
			*/
		}
	}
	//cout << " " << endl;

	vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);
	return trajectories[best_idx];
}

TRAJECTORY EgoVehicle::GenerateTrajectoryForState(EgoVehicleState state, map<int, VEHICLE> &predictions)
{
	if(state == EgoVehicleState::LaneChangeLeft)
	{
		return GenerateLaneChangeTrajectory(ChangeLaneDirection::Left, predictions);
	}
	else if(state == EgoVehicleState::LaneChangeRight)
	{
		return GenerateLaneChangeTrajectory(ChangeLaneDirection::Right, predictions);
	}
	else
	{
		return GenerateKeepLaneTrajectory(predictions);
	}
}

TRAJECTORY EgoVehicle::FillTrajectoryPoints(TRAJECTORY &result)
{
		/*
		int prev_size = _previous_path_x.size();

		if(prev_size > 0)
		{
			_car_s = _end_path_s;
		}


		_close = false;
		_tooClose = false;

		double speedDiffRate = 1.0;

		for(int i = 0; i < _sensor_fusion.size(); i++)
		{
			float d = _sensor_fusion[i][6];
		    if(d < (_road.LaneWidth*(result.targetLane+1)) && d > (_road.LaneWidth*result.targetLane))
		    {
		    	double vx = _sensor_fusion[i][3];
		        double vy = _sensor_fusion[i][4];
		        double check_speed = sqrt(vx*vx+vy*vy);
		        double check_car_s = _sensor_fusion[i][5];

		        check_car_s+=((double)prev_size*delta_t*check_speed);
		        if((check_car_s > _car_s) && ( (check_car_s-_car_s) < planXHorizon) )
		        {
		        	//speedDiffRate = 10*(planXHorizon-(check_car_s-_car_s))/planXHorizon;
		        	_tooClose = true;
		        }
		    }
		}

		map<int, VEHICLE> predictions = _predictor.GetPredictions((double)prev_size*delta_t);
		for(map<int, VEHICLE>::iterator it = predictions.begin(); it != predictions.end(); ++it)
		{
			double d = it->second.d;
			double check_car_s = it->second.s;

			if(d < (_road.LaneWidth*(result.targetLane+1)) && d > (_road.LaneWidth*result.targetLane))
			{
				if((check_car_s > _car_s) && ( (check_car_s-_car_s) < planXHorizon) )
				{
					//speedDiffRate = 10*(planXHorizon-(check_car_s-_car_s))/planXHorizon;
					_close = true;
				}
			}
		}

		if(_close )
		{
			result.targetVel -= speedDiffRate*_veloctyChangeRate;
		}
		else if(result.targetVel < _maxVelocity)
		{
			result.targetVel += speedDiffRate*_veloctyChangeRate;
		}
		*/


		int prev_size = _previous_path_x.size();

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

		vector<double> next_wp0 = calHelper::getXY(_car_s+planXHorizon,(_road.LaneWidth/2+_road.LaneWidth*_targetLane),_map_waypoints_s, _map_waypoints_x, _map_waypoints_y);
		vector<double> next_wp1 = calHelper::getXY(_car_s+planXHorizon*2,(_road.LaneWidth/2+_road.LaneWidth*_targetLane),_map_waypoints_s, _map_waypoints_x, _map_waypoints_y);
		vector<double> next_wp2 = calHelper::getXY(_car_s+planXHorizon*3,(_road.LaneWidth/2+_road.LaneWidth*_targetLane),_map_waypoints_s, _map_waypoints_x, _map_waypoints_y);

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

		return result;
}

TRAJECTORY EgoVehicle::GenerateKeepLaneTrajectory(map<int, VEHICLE> &predictions)
{
	TRAJECTORY result;
	result.targetVel = _targetVel;
	result.targetLane = _targetLane;

	double speedDiffRateToFollow = _defaultSpeedDiffRate;
	VEHICLE vhAhead = getVehicleAhead(result.targetLane, _car_s, predictions);
	if(vhAhead.id != -1)
	{
		speedDiffRateToFollow = getSpeedDiffRateToFollow(vhAhead.s, _car_s, result);
	}

	result.targetVel = calcVelocity(speedDiffRateToFollow);

	/*
	for(map<int, VEHICLE>::iterator it = predictions.begin(); it != predictions.end(); ++it)
	{
		double d = it->second.d;
		double check_car_s = it->second.s;

		if(d < (_road.LaneWidth*(result.targetLane+1)) && d > (_road.LaneWidth*result.targetLane))
		{
			if((check_car_s > _car_s) && ( (check_car_s-_car_s) < 1.3*planXHorizon) )
			{
				_close = true;
				double newvcte = planXHorizon-(check_car_s-_car_s);

				speedDiffRate = -0.005*newvcte-4.5*(newvcte-_vcte);

				if(speedDiffRate > _maxspeedDiffRate)
				{
					speedDiffRate = _maxspeedDiffRate;
				}
				if(speedDiffRate < -_maxspeedDiffRate)
				{
					speedDiffRate = -_maxspeedDiffRate;
				}

				_vcte = planXHorizon-(check_car_s-_car_s);
			}
			else
			{
				_vcte = 0.0;
			}
		}
	}

	if(_close )
	{
		cout << "speedDiffRate= " << speedDiffRate << endl;
		result.targetVel += speedDiffRate*_veloctyChangeRate;
		if(result.targetVel > _maxVelocity)
		{
			result.targetVel = _maxVelocity;
		}
		if(result.targetVel < _minVelocity)
		{
			result.targetVel = _minVelocity;
		}
	}
	else if(result.targetVel < _maxVelocity)
	{
		cout << " " << endl;
		result.targetVel += _veloctyChangeRate;
	}
	*/

	result.found = true;
	return FillTrajectoryPoints(result);
}

TRAJECTORY EgoVehicle::GenerateLaneChangeTrajectory(ChangeLaneDirection direction, map<int, VEHICLE> &predictions)
{
	int lane = _targetLane;
	string dir;
	if(direction == ChangeLaneDirection::Left)
	{
		lane--;
		dir = "left";
	}
	if(direction == ChangeLaneDirection::Right)
	{
		lane++;
		dir = "right";
	}

	TRAJECTORY result;
	result.targetVel = _targetVel;
	result.targetLane = lane;

	double speedDiffRateToFollow = 3.0;
	VEHICLE vhAhead = getVehicleAhead(result.targetLane, _car_s, predictions);
	VEHICLE vhBehind = getVehicleBehind(result.targetLane, _car_s, predictions);

	double dsa = abs(0.5);
	if(vhAhead.id != -1 && abs(vhAhead.s-_car_s) < _safeDistanceAheadForChange)
	{
		//cout << dir << " changing is not allowed" << endl;
		return result;
	}
	if(vhBehind.id != -1 && abs(vhBehind.s-_car_s) < _safeDistanceBehindForChange)
	{
		//cout << dir << " changing is not allowed" << endl;
		return result;
	}
	//cout << dir << " " << endl;

	if(vhAhead.id != -1)
	{
		speedDiffRateToFollow = getSpeedDiffRateToFollow(vhAhead.s, _car_s, result);
	}

	result.targetVel = calcVelocity(speedDiffRateToFollow);

	/*
	_close = false;
	double speedDiffRate = 1.0;
	for(map<int, VEHICLE>::iterator it = predictions.begin(); it != predictions.end(); ++it)
	{
		double d = it->second.d;
		double check_car_s = it->second.s;
		double check_car_v = it->second.v;

		if(d < (_road.LaneWidth*(result.targetLane+1)) && d > (_road.LaneWidth*result.targetLane))
		{
			if(abs(check_car_s-_car_s) < planXHorizon/2)
			{
				//cout << dir << " changing is not allowed" << endl;
				return result;
			}

			if((check_car_s > _car_s) && ( (check_car_s-_car_s) < planXHorizon) )
			{
				_close = true;
			}
		}

		//cout << " " << endl;
	}

	if(_close )
	{
		result.targetVel -= speedDiffRate*_veloctyChangeRate;
	}
	else if(result.targetVel < _maxVelocity)
	{
		result.targetVel += speedDiffRate*_veloctyChangeRate;
	}
	*/

	result.found = true;
	return FillTrajectoryPoints(result);
}

vector<EgoVehicleState> EgoVehicle::getSuccessorStates()
{
	vector<EgoVehicleState> states;
	//states.push_back(EgoVehicleState::KeepLane);

	if(_currentState == EgoVehicleState::LaneChanging)
	{
		double ddiff = abs(_car_d - ((double)_targetLane+0.5)*_road.LaneWidth);
		if(ddiff < 0.4)
		{
			states.push_back(EgoVehicleState::KeepLane);
			return states;
		}
		else
		{
			states.push_back(EgoVehicleState::LaneChanging);
			return states;
		}
	}

	if(_currentState == EgoVehicleState::LaneChangeLeft || _currentState == EgoVehicleState::LaneChangeRight)
	{
		states.push_back(EgoVehicleState::LaneChanging);
		return states;
	}

	if(_targetLane == 0)
	{
		states.push_back(EgoVehicleState::KeepLane);
		states.push_back(EgoVehicleState::LaneChangeRight);
	}
	else if(_targetLane == _road.LanesAvailable-1)
	{
		states.push_back(EgoVehicleState::KeepLane);
		states.push_back(EgoVehicleState::LaneChangeLeft);
	}
	else
	{
		states.push_back(EgoVehicleState::KeepLane);
		states.push_back(EgoVehicleState::LaneChangeLeft);
		states.push_back(EgoVehicleState::LaneChangeRight);
	}
	//_car_d

	return states;
}

double EgoVehicle::calcCost(TRAJECTORY trajectory)
{
	return _velocityWeight*(_maxVelocity-trajectory.targetVel)
			+ _maxSpaceAheadWeight*(farPlanXHorizon-trajectory.maxSpaceAheadAvailable)
			+ _spaceAheadWeight*(farPlanXHorizon-trajectory.spaceAheadAvailable)
			+ _stateWeight*((double)trajectory.state);
}

VEHICLE EgoVehicle::getVehicleAhead(int lane, double s, map<int, VEHICLE> &predictions)
{
	VEHICLE result;
	result.id = -1;
	result.s = numeric_limits<double>::max();

	for(map<int, VEHICLE>::iterator it = predictions.begin(); it != predictions.end(); ++it)
	{
		double d = it->second.d;
		vector<int> vehLanes = it->second.lanes;

		//if(d < (_road.LaneWidth*(lane+1)) && d > (_road.LaneWidth*lane))
		if(find(vehLanes.begin(), vehLanes.end(), lane) != vehLanes.end())
		{
			if(it->second.s > s && it->second.s < result.s)
			{
				result = it->second;
			}
		}
	}

	return result;
}

VEHICLE EgoVehicle::getVehicleBehind(int lane, double s, map<int, VEHICLE> &predictions)
{
	VEHICLE result;
	result.id = -1;
	result.s = numeric_limits<double>::min();

	for(map<int, VEHICLE>::iterator it = predictions.begin(); it != predictions.end(); ++it)
	{
		double d = it->second.d;
		vector<int> vehLanes = it->second.lanes;

		//if(d < (_road.LaneWidth*(lane+1)) && d > (_road.LaneWidth*lane))
		if(find(vehLanes.begin(), vehLanes.end(), lane) != vehLanes.end())
		{
			if(it->second.s < s && it->second.s > result.s)
			{
				result = it->second;
			}
		}
	}

	return result;
}

double EgoVehicle::getSpeedDiffRateToFollow(double carToFollowS, double s, TRAJECTORY traj)
{
	double speedDiffRate = _defaultSpeedDiffRate;
	if((carToFollowS > s) && ( (carToFollowS-s) < 1.2*_followDistance) )
	{
		double newvcte = _followDistance-(carToFollowS-s);

		speedDiffRate = -0.0018*newvcte-3.5*(newvcte-_vcte);

		if(speedDiffRate > _maxspeedDiffRate)
		{
			speedDiffRate = _maxspeedDiffRate;
		}
		if(speedDiffRate < -_maxspeedDiffRate)
		{
			speedDiffRate = -_maxspeedDiffRate;
		}

		traj.vcte = _followDistance-(carToFollowS-_car_s);
	}
	else
	{
		traj.vcte = 0.0;
	}

	return speedDiffRate;
}


double EgoVehicle::calcVelocity(double speedDiffRate)
{
	double result = _targetVel + speedDiffRate*_veloctyChangeRate;

	if(result > _maxVelocity)
	{
		result = _maxVelocity;
	}
	if(result < _minVelocity)
	{
		result = _minVelocity;
	}

	return result;
}

double EgoVehicle::calcMaxSpaceAheadAvailable(int lane, map<int, VEHICLE> &predictions)
{
	double result = 0;
	vector<int> lanesAvailable = {lane};
	if(lane == 0)
	{
		lanesAvailable.push_back(lane+1);
	}
	else if(lane == _road.LanesAvailable-1)
	{
		lanesAvailable.push_back(lane-1);
	}
	else
	{
		lanesAvailable.push_back(lane+1);
		lanesAvailable.push_back(lane-1);
	}

	for(vector<int>::iterator it = lanesAvailable.begin(); it != lanesAvailable.end(); ++it)
	{
		double currentValue = calcSpaceAheadAvailable(*it, predictions);

		if(currentValue > result)
		{
			result = currentValue;
		}
	}

	return result;
}

double EgoVehicle::calcSpaceAheadAvailable(int lane, map<int, VEHICLE> &predictions)
{
	double result = 0;
	VEHICLE vhAhead = getVehicleAhead(lane, _car_s, predictions);
	VEHICLE vhBehind = getVehicleAhead(lane, _car_s, predictions);

	if(vhAhead.id != -1 && abs(vhAhead.s-_car_s) < _safeDistanceAheadForChange)
	{
		result = 0;
	}
	else if(vhBehind.id != -1 && abs(vhBehind.s-_car_s) < _safeDistanceBehindForChange)
	{
		result = 0;
	}
	else
	{
		result = min(farPlanXHorizon, vhAhead.s - _car_s);
	}

	return result;
}
