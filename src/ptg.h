#pragma once
#include <vector>
#include <vehicle.h>
#include "constants.h"

#include "cost_function.h"

//Polynomial Trajectory Generation
class ptg
{

public:

	ptg();
	ptg(std::vector<float> start);
	std::vector<float> Find_Trajectory(std::vector<float> start_s, std::vector<float> start_d,
									   int target_vehicle, std::vector<float> delta, float time,
		                               std::vector<vehicle> predictions);
	~ptg();

private: 

	goal_t Perturb_Goal(std::vector<float> goal_s, std::vector<float> goal_d, float T);
	std::vector<float> JMT(std::vector<float> start, std::vector<float> goal, float T);
	std::vector<float> start_state;
	cost_function cost_func;

};
