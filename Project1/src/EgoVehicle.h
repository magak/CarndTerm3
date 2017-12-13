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

enum class EgoVehicleState { KeepLane, LaneChangeLeft, LaneChangeRight, LaneChanging};
enum class ChangeLaneDirection { Left, Right };

/*
 * Trajectory objects
 */
struct TRAJECTORY
{
	int targetLane;
	double targetVel;
	vector<double> next_x_vals; // x coordinates of points
	vector<double> next_y_vals; // y coordinates of points
	double vcte = 0.0; // CTE value for PD controller of distance to car ahead
	bool found = false; // whether the trajectory is possible
	double maxSpaceAheadAvailable = 0; // the maximal space ahead for all available lanes for the vehicle
	double spaceAheadAvailable = 0; // Returns the space ahead for the current lane
	EgoVehicleState state;
};

const double delta_t = 0.02;
const double speedMPHRatio = 2.24;

/*
 * Ego vehicle object
 */
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

	/*
	 * Processing telemetry from the simulator
	 */
	void ProcessInputData(
			double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
			nlohmann::basic_json<>& previous_path_x_orig, nlohmann::basic_json<>& previous_path_y_orig,
			double end_path_s, double end_path_d, vector<vector<double>> sensor_fusion,
			vector<double> &next_x_vals, vector<double> &next_y_vals
			);

private:

	// data from the simulator
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

	// current state
	EgoVehicleState _currentState;

	// target velocity
	double _targetVel = 0;

	// the size of previous points vector
	int _prev_size = 0;

	/* max and min velocty */
	double _maxVelocity = 49.1;
	double _minVelocity = 3.1;

	/* maximal speed change rate */
	double _maxspeedDiffRate = 7.3;

	/* velocty change rate*/
	double _veloctyChangeRate = .224;

	// current lane
	int _currentLane;

	// target lane
	int _targetLane;

	// road parameters
	ROAD_CONFIGURATION _road;
	// predictor
	Predictor _predictor;

	/* weights */
	double _velocityWeight = 1.0;
	double _maxSpaceAheadWeight = 0.03;
	double _spaceAheadWeight = 0.01;
	double _stateWeight = 0.1;

	// default speed change rate
	double _defaultSpeedDiffRate = 3.0;

	// follow distance rate parameters
	double _followDistance = 17.0;
	double _followDistanceChange = 0.2;
	double _followDistanceMin = 14.0;
	double _followDistanceMax = 34.0;

	// safe distance ahead for lane changing
	double _safeDistanceBehindForChange = 12.4;

	// safe distance behind for lane changing
	double _safeDistanceAheadForChange = 12;

	// path points count
	int pathPointCount = 50;

	// planing horizon distance
	double planXHorizon = 30.0;
	double farPlanXHorizon = 60.0;

	// previous CTE of distance PD controller
	double _vcte = 0.0;

	bool _tooClose = false;
	bool _close = false;

	vector<double> _map_waypoints_x;
	vector<double> _map_waypoints_y;
	vector<double> _map_waypoints_s;
	vector<double> _map_waypoints_dx;
	vector<double> _map_waypoints_dy;

	/*
	 * Calculating the best trajectory
	 */
	TRAJECTORY GetBestTrajectory();

	/*
	 * Generation trajectory for the specified state
	 */
	TRAJECTORY GenerateTrajectoryForState(EgoVehicleState state, map<int, VEHICLE> &predictions);

	/*
	 * Generating trajectory for KeepLane state
	 */
	TRAJECTORY GenerateKeepLaneTrajectory(map<int, VEHICLE> &predictions);

	/*
	 * Generating trajectory for ChangeLane state
	 */
	TRAJECTORY GenerateLaneChangeTrajectory(ChangeLaneDirection direction, map<int, VEHICLE> &predictions);

	/*
	 * Calculating path points for trajectory
	 */
	TRAJECTORY FillTrajectoryPoints(TRAJECTORY &result);

	/*
	 * Returns the list of possible next state
	 */
	vector<EgoVehicleState> getSuccessorStates();

	/*
	 * Cost function
	 */
	double calcCost(TRAJECTORY trajectory);

	/*
	 * given the predictions, returns the vehicle on the specified lane for specified s position
	 */
	VEHICLE getVehicleAhead(int lane, double s, map<int, VEHICLE> &predictions);

	/*
	 * given the predictions, returns the vehicle on the specified lane for specified s position
	 */
	VEHICLE getVehicleBehind(int lane, double s, map<int, VEHICLE> &predictions);

	/*
	 * Calculates how much the speed of the vehicle changes
	 */
	double getSpeedDiffRateToFollow(double carToFollowS, double s, TRAJECTORY traj);

	/*
	 * Calculation of velocity for the specified speed change rate
	 */
	double calcVelocity(double speedDiffRate);

	/*
	 * Returns the maximal space ahead for all available lanes for the vehicle
	 */
	double calcMaxSpaceAheadAvailable(int lane, map<int, VEHICLE> &predictions);

	/*
	 * Returns the space ahead for the current lane
	 */
	double calcSpaceAheadAvailable(int lane, map<int, VEHICLE> &predictions);
};

#endif /* EGOVEHICLE_H_ */
