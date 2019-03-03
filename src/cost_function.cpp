#include "cost_function.h"
#include "math.h"
#include "float.h"

std::vector<float> differentiate(std::vector<float> coefficients)
{

	//Calculates the derivative of a polynomial and returns
	//the corresponding coefficients.
	std::vector<float> new_cos;
	for (int id = 1; id < coefficients.size(); id++) 
	{
		new_cos.push_back((id) * coefficients[id]);
	}
	
    return new_cos;
}
//-----------------------------------------------------------------------------------
float pol_evaluation(std::vector<float> coeff, float t)
{
	float value = 0.0f;
	for (int id = 0; id < coeff.size(); id++)
	{
		value += coeff[id] * (pow(t, id));
	}

	return value;
}
//-----------------------------------------------------------------------------------
float nearest_approach(trajectory_t trajectory, vehicle prediction)
{
	std::vector<float> s_traj = trajectory.coeff_s;
	std::vector<float> d_traj = trajectory.coeff_d;
	float T = trajectory.t;

	//prediction.state_in()
	float closest = FLT_MAX;
	for (int i = 0; i < 100; i++) 
	{
		float t = (float)(i) / 100 * T;

		float cur_s = pol_evaluation(s_traj, t);
		float cur_d = pol_evaluation(d_traj, t);

		std::vector<float> targ = prediction.state_in(t);
		float targ_s = targ[0];
		float targ_d = targ[3];

		double current_dist = sqrt(pow(cur_s - targ_s, 2) + pow(cur_d - targ_d, 2));

		if (current_dist < closest) 
		{
			closest = current_dist;
		}
	}
	return closest;
}
//-----------------------------------------------------------------------------------
float nearest_approach_to_any_vehicle(trajectory_t trajectory, std::vector<vehicle> predictions)
{
	//Calculates the closest distance to any vehicle during a trajectory.
	float closest = FLT_MAX;
	for (int id_vehicle=0;id_vehicle< predictions.size();id_vehicle++)
	{
		float d = nearest_approach(trajectory, predictions[id_vehicle]);
		if (d < closest)
		{
			closest = d;
		}
	}

	return closest;
}
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
	/*S = [f(T) for f in get_f_and_N_derivatives(s, 2)]
	cost = 0
	for actual, expected, sigma in zip(S, s_targ, SIGMA_S) :
		diff = float(abs(actual - expected))
		cost += logistic(diff / sigma)
		return cost*/

	return 1;
}
//-----------------------------------------------------------------------------------
float collision_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	//Binary cost function which penalizes collisions.
	float nearest = nearest_approach_to_any_vehicle(trajectory, predictions);
	if (nearest < 2.0f * VEHICLE_RADIUS)
	{
		return 1.0f;
	}
	else 
	{ 
		return 0.0f;
	}
}
//-----------------------------------------------------------------------------------
float buffer_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	//	Penalizes getting close to other vehicles.	
	float nearest = nearest_approach_to_any_vehicle(trajectory, predictions);
	return logistic(2 * VEHICLE_RADIUS / nearest);
}
//-----------------------------------------------------------------------------------
float efficiency_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{	
	//Rewards high average speeds.
	float avg_v = (float)(pol_evaluation(trajectory.coeff_s, trajectory.t)) / trajectory.t;
    float targ_s = predictions[target_vehicle].state_in(trajectory.t)[0];

	float targ_v = (float)(targ_s) / trajectory.t;
	return logistic(2 * float(targ_v - avg_v) / avg_v);
}
//-----------------------------------------------------------------------------------
float total_accel_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	//s, d, t = traj;

	std::vector<float> s_dot = differentiate(trajectory.coeff_s);
	std::vector<float> s_d_dot = differentiate(s_dot);
	//a = to_equation(s_d_dot);
	float total_acc = 0;
	float dt = T / 100.0f;
	float t = 0;
	
	//for i in range(100) :
	for (int i=0;i<100;i++)
	{
		t = dt * i;
		float acc = pol_evaluation(s_d_dot, t);// a(t);
		total_acc += abs(acc*dt);
	}
	float acc_per_second = total_acc / T;
	return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}
//-----------------------------------------------------------------------------------


cost_function::cost_function()
{
	functions_map["total_accel_cost_priv"] = total_accel_cost_priv;
	//std::string s("add");

	trajectory_t trajectory;
	int target_vehicle=0;
	std::vector<float> delta;
	float goal_t=5.0;
	std::vector<vehicle> predictions;
	vehicle veh;
	std::vector<float> state{ 0, 10, 0, 0, 0, 0 };
	veh.start_state = state;
	predictions.push_back(veh);
	std::vector<float> coeff_s{ 10.0, 10.0, 0.0, -3.7037037,1.85185185,-0.24691358 };
	std::vector<float> coeff_d{ 4.0, 0.0, 0.0, -1.48148148, 0.74074074, -0.09876543 };

	trajectory.coeff_s = coeff_s;
	trajectory.coeff_d = coeff_d;
	trajectory.t = 3.0f;

	std::string cost_functions = "total_accel_cost_priv";
	
	int res = functions_map[cost_functions](trajectory, target_vehicle, delta, goal_t, predictions);
}
 

cost_function::~cost_function()
{
}
