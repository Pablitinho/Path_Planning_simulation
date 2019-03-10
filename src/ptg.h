#pragma once
#include <vector>
#include <vehicle.h>
#include "constants.h"
#include <vector>
#include "cost_function.h"

typedef struct
{
	float x;
	float y;
}point_2d_t;

//Polynomial Trajectory Generation
class ptg
{

public:

	ptg();
	ptg(std::vector<float> start);
	trajectory_t Find_Trajectory(std::vector<float> start_s, std::vector<float> start_d,
									   int target_vehicle, std::vector<float> delta, float time,
		                               std::vector<vehicle> predictions);

	std::vector<point_2d_t> Create_Trajectory(trajectory_t best_traj);

	~ptg();

private: 

	int best_trajectory(std::vector<trajectory_t> trajectories, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions);
	goal_t Perturb_Goal(std::vector<float> goal_s, std::vector<float> goal_d, float T);
	std::vector<float> JMT(std::vector<float> start, std::vector<float> goal, float T);
	std::vector<float> start_state;
	cost_function cost_func;

};

