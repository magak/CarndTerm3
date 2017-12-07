/*
 * EgoVehicle.h
 *
 *  Created on: Dec 1, 2017
 *      Author: magak
 */

#ifndef EGOVEHICLE_H_
#define EGOVEHICLE_H_

#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <random>
#include "json.hpp"
#include "Road.h"
#include "Predictor.h"

using namespace std;

struct TRAJECTORY
{
	int targetLane;
	double targetVel;
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	bool found = false;
};

enum class EgoVehicleState { KeepLane, LaneChangeLeft, LaneChangeRight};

enum class ChangeLaneDirection { Left, Right };

const double delta_t = 0.02;
const double speedMPHRatio = 2.24;

class EgoVehicle {
public:
	/**
	* Constructor
	*/
	EgoVehicle(int currentLane, ROAD_CONFIGURATION road,
			vector<double> map_waypoints_x,
			vector<double> map_waypoints_y,
			vector<double> map_waypoints_s,
			vector<double> map_waypoints_dx,
			vector<double> map_waypoints_dy,
			EgoVehicleState state = EgoVehicleState::KeepLane);

	void ProcessInputData(
			double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
			nlohmann::basic_json<>& previous_path_x_orig, nlohmann::basic_json<>& previous_path_y_orig,
			double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion,
			vector<double> &next_x_vals, vector<double> &next_y_vals
			);

private:

	double _car_x;
	double _car_y;
	double _car_s;
	double _car_d;
	double _car_yaw;
	double _car_speed;
	vector<double> _previous_path_x;
	vector<double> _previous_path_y;
	double _end_path_s;
	double _end_path_d;
	vector<vector<double>> _sensor_fusion;

	EgoVehicleState _currentState;

	double _targetVel = 0;

	/* max velocty */
	double _maxVelocity = 49.5;

	/* velocty change rate*/
	double _veloctyChangeRate = .224;

	int _currentLane;

	int _targetLane;

	ROAD_CONFIGURATION _road;
	Predictor _predictor;

	int pathPointCount = 50;

	double planXHorizon = 30.0;

	bool _tooClose = false;
	bool _close = false;
	bool _lowFar = false;

	vector<double> _map_waypoints_x;
	vector<double> _map_waypoints_y;
	vector<double> _map_waypoints_s;
	vector<double> _map_waypoints_dx;
	vector<double> _map_waypoints_dy;

	TRAJECTORY GetBestTrajectory();

	TRAJECTORY GenerateTrajectoryForState(EgoVehicleState state);

	TRAJECTORY GenerateKeepLaneTrajectory();

	TRAJECTORY GenerateLaneChangeTrajectory(ChangeLaneDirection direction);

	TRAJECTORY GenerateTrajectoryForLane(int lane);

	vector<EgoVehicleState> getSuccessorStates();

	double calcCost(TRAJECTORY trajectory);
};

#endif /* EGOVEHICLE_H_ */
