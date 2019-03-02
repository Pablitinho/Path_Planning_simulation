#include "cost_function.h"
#include "math.h"
//-----------------------------------------------------------------------------------
float logistic(float x)
{
	
	//A function that returns a value between 0 and 1 for x in the
	//range[0, infinity] and -1 to 1 for x in the range[-infinity, infinity].
	//Useful for cost functions.
	return 2.0 / (1 + exp(-x)) - 1.0;
}
//-----------------------------------------------------------------------------------
float cost_function::time_diff_cost(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float goal_t, std::vector<vehicle> predictions)
{
		//Penalizes trajectories that span a duration which is longer or
		//shorter than the duration requested.

	//	float t = traj
	//	return logistic(float(abs(t - T)) / T)
	return 1.0f;
}
//-----------------------------------------------------------------------------------
float time_diff_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	//Penalizes trajectories that span a duration which is longer or
//shorter than the duration requested.

	float t = trajectory.t;
	return logistic(float(fabsf(t - T)) / T);

}
//-----------------------------------------------------------------------------------
float s_diff_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{

	//Penalizes trajectories whose s coordinate(and derivatives)
	//differ from the goal.

	std::vector<float> s = trajectory.coeff_s;
	std::vector<float> target = predictions[target_vehicle].state_in(T);

	for (int id = 0; id < target.size(); id++)
	{
		target[id] += delta[id];
	}
	std::vector<float> s_target{ target[0],target[1],target[2] };

	//target = predictions[target_vehicle].state_in(T)
	//target = list(np.array(target) + np.array(delta))
	//s_targ = target[:3]
	S = [f(T) for f in get_f_and_N_derivatives(s, 2)]
	cost = 0
	for actual, expected, sigma in zip(S, s_targ, SIGMA_S) :
		diff = float(abs(actual - expected))
		cost += logistic(diff / sigma)
		return cost
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------


cost_function::cost_function()
{
	functions_map["time_diff_cost"] = time_diff_cost_priv;
	//std::string s("add");

	trajectory_t trajectory;
	int target_vehicle=0;
	std::vector<float> delta;
	float goal_t=0;
	std::vector<vehicle> predictions;
	std::string cost_functions = "time_diff_cost";
	
	int res = functions_map[cost_functions](trajectory, target_vehicle, delta, goal_t, predictions);
}


cost_function::~cost_function()
{
}
