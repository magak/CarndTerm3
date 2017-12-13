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

/*
 * Processing telemetry from the simulator
 */
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

	// conveying data from sensor fusion to predictor
	_predictor.ProcessData(_sensor_fusion);

	_followDistance += _followDistanceChange;

	// periodically changing the distance to the vehicle ahead so there is opportunity
	// to consider several ways to act
	if(_followDistance > _followDistanceMax || _followDistance < _followDistanceMin)
	{
		_followDistanceChange = (-1)*_followDistanceChange;
	}

  	int limit_prev_size = previous_path_x_orig.size();
  	_prev_size = limit_prev_size;

  	// only 5 points from the previous path will be taken into account for smoothing the path
  	// the rest will be generated so the car can react more adequately to the situation on the road
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
}

/*
 * Calculating the best trajectory
 */
TRAJECTORY EgoVehicle::GetBestTrajectory()
{
	vector<TRAJECTORY> trajectories;
	double cost;
	vector<double> costs;
	vector<EgoVehicleState> states = getSuccessorStates();

	map<int, VEHICLE> predictions = _predictor.GetPredictions((double)_prev_size*delta_t);

	string statesStr[] = { "KeepLane", "LaneChangeLeft", "LaneChangeRight", "LaneChanging"};
	for(vector<EgoVehicleState>::iterator it = states.begin(); it != states.end(); ++it)
	{
		TRAJECTORY trajectory = GenerateTrajectoryForState(*it, predictions);
		trajectory.state = *it;

		if(trajectory.found)
		{
			// calculation of trajectory parameters for the cost function
			trajectory.maxSpaceAheadAvailable = calcMaxSpaceAheadAvailable(trajectory.targetLane, predictions);
			trajectory.spaceAheadAvailable = calcSpaceAheadAvailable(trajectory.targetLane, predictions);
			cost = calcCost(trajectory);
			costs.push_back(cost);
			trajectories.push_back(trajectory);
		}
	}

	// the trajectory with the lowes cost function value is the best
	vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx = distance(begin(costs), best_cost);
	return trajectories[best_idx];
}

/*
 * Generation trajectory for the specified state
 */
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

/*
 * Calculating path points for trajectory
 */
TRAJECTORY EgoVehicle::FillTrajectoryPoints(TRAJECTORY &result)
{
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
		// splining the points
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

/*
 * Generating trajectory for KeepLane state
 */
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

	result.found = true;
	return FillTrajectoryPoints(result);
}

/*
 * Generating trajectory for ChangeLane state
 */
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

	double speedDiffRateToFollow = _defaultSpeedDiffRate;
	VEHICLE vhAhead = getVehicleAhead(result.targetLane, _car_s, predictions);
	VEHICLE vhBehind = getVehicleBehind(result.targetLane, _car_s, predictions);

	// There should be enough space for changing the lane
	if(vhAhead.id != -1 && abs(vhAhead.s-_car_s) < _safeDistanceAheadForChange)
	{
		return result;
	}
	if(vhBehind.id != -1 && abs(vhBehind.s-_car_s) < _safeDistanceBehindForChange)
	{
		return result;
	}

	if(vhAhead.id != -1)
	{
		speedDiffRateToFollow = getSpeedDiffRateToFollow(vhAhead.s, _car_s, result);
	}

	result.targetVel = calcVelocity(speedDiffRateToFollow);

	result.found = true;
	return FillTrajectoryPoints(result);
}

/*
 * Returns the list of possible next state
 */
vector<EgoVehicleState> EgoVehicle::getSuccessorStates()
{
	vector<EgoVehicleState> states;

	if(_currentState == EgoVehicleState::LaneChanging)
	{
		// unless the target lane is reached the next only is LaneChanging
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

	return states;
}

/*
 * Cost function
 */
double EgoVehicle::calcCost(TRAJECTORY trajectory)
{
	return _velocityWeight*(_maxVelocity-trajectory.targetVel) // trajectory velocity part
			+ _maxSpaceAheadWeight*(farPlanXHorizon-trajectory.maxSpaceAheadAvailable) // all available lanes space ahead part
			+ _spaceAheadWeight*(farPlanXHorizon-trajectory.spaceAheadAvailable) // space ahead for current lane part
			+ _stateWeight*((double)trajectory.state); // state part: (e.g. if other parts of the cost function equals the KeepLane is preferable )
}

/*
 * given the predictions, returns the vehicle on the specified lane for specified s position
 */
VEHICLE EgoVehicle::getVehicleAhead(int lane, double s, map<int, VEHICLE> &predictions)
{
	VEHICLE result;
	result.id = -1;
	result.s = numeric_limits<double>::max();

	for(map<int, VEHICLE>::iterator it = predictions.begin(); it != predictions.end(); ++it)
	{
		double d = it->second.d;
		vector<int> vehLanes = it->second.lanes;

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

/*
 * given the predictions, returns the vehicle on the specified lane for specified s position
 */
VEHICLE EgoVehicle::getVehicleBehind(int lane, double s, map<int, VEHICLE> &predictions)
{
	VEHICLE result;
	result.id = -1;
	result.s = numeric_limits<double>::min();

	for(map<int, VEHICLE>::iterator it = predictions.begin(); it != predictions.end(); ++it)
	{
		double d = it->second.d;
		vector<int> vehLanes = it->second.lanes;

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

/*
 * Calculates how much the speed of the vehicle changes
 */
double EgoVehicle::getSpeedDiffRateToFollow(double carToFollowS, double s, TRAJECTORY traj)
{
	double speedDiffRate = _defaultSpeedDiffRate;
	if((carToFollowS > s) && ( (carToFollowS-s) < 1.2*_followDistance) )
	{
		double newvcte = _followDistance-(carToFollowS-s);

		// PD-controller for controlling the distance to the car ahead
		speedDiffRate = -0.0016*newvcte-3.3*(newvcte-_vcte);

		// speed constraints
		if(speedDiffRate > _maxspeedDiffRate)
		{
			speedDiffRate = _maxspeedDiffRate;
		}
		if(speedDiffRate < -_maxspeedDiffRate)
		{
			speedDiffRate = -_maxspeedDiffRate;
		}

		// remembering the CTE for d part in the next iteration
		traj.vcte = _followDistance-(carToFollowS-_car_s);
	}
	else
	{
		traj.vcte = 0.0;
	}

	return speedDiffRate;
}

/*
 * Calculation of velocity for the specified speed change rate
 */
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

/*
 * Returns the maximal space ahead for all available lanes for the vehicle
 */
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

/*
 * Returns the space ahead for the current lane
 */
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

