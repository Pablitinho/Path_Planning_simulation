#pragma once
#include <vector>
#include "vehicle.h"
#include <map>
#include "constants.h"

typedef float(*FnPtr)(trajectory_t, int, std::vector<float>,float, std::vector<vehicle>);

class cost_function
{

public:

	cost_function();
	~cost_function();
	//-----------------------------------------------------------------------------------
	float time_diff_cost(trajectory_t trajectory, int target_vehicle, 
		                               std::vector<float> delta, float goal_t,
		                               std::vector<vehicle> predictions);

	float estimate_all_costs(trajectory_t trajectory, int target_vehicle,
							std::vector<float> delta, float T,
							std::vector<vehicle> predictions);
	static float pol_evaluation(std::vector<float> coeff, float t);
	//-----------------------------------------------------------------------------------
	//-----------------------------------------------------------------------------------
	//-----------------------------------------------------------------------------------
	//-----------------------------------------------------------------------------------
	//-----------------------------------------------------------------------------------
	//-----------------------------------------------------------------------------------
	std::map<std::string, FnPtr> functions_map;
	std::vector<std::string> functions_name;
private: 
							  //0   //1   //2   //3   //4    //5  //6    //7  //8   //9   //10   //11
	float weight_cost[12] = { 0.8f, 0.8f, 0.8f, 0.8f, 0.3f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.5f };

};

