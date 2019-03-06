#include "cost_function.h"
#include <cmath>
#include "float.h"
#include <iostream>
//-----------------------------------------------------------------------------------
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
float cost_function::pol_evaluation(std::vector<float> coeff, float t)
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

		float cur_s = cost_function::pol_evaluation(s_traj, t);
		float cur_d = cost_function::pol_evaluation(d_traj, t);

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
	std::vector<float> target = predictions[target_vehicle].state_in(trajectory.t);

	for (int id = 0; id < target.size(); id++)
	{
		target[id] += delta[id];
	}
	std::vector<float> s_target{ target[0],target[1],target[2] };

	int N = 2;
	std::vector<float> list_diff;
	std::vector<float> coeff=s;
	list_diff.push_back(cost_function::pol_evaluation(s, trajectory.t));

	for (int id = 0; id < N; id++) 
	{
		coeff = differentiate(coeff);

		list_diff.push_back(cost_function::pol_evaluation(coeff, trajectory.t));
	}
	float cost = 0.0f;

	for (int id = 0; id < list_diff.size(); id++)
	{
		float actual = list_diff[id];
		float expected = s_target[id];
		float sigma = sigma_s[id];
		float diff = fabsf(actual - expected);
		cost += logistic(diff / sigma);
	}

	return cost;
}
//-----------------------------------------------------------------------------------
float d_diff_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	//Penalizes trajectories whose d coordinate(and derivatives)
	//differ from the goal.
	T = trajectory.t;

	std::vector<float> d_dot_coeffs = differentiate(trajectory.coeff_d);
	std::vector<float> d_ddot_coeffs = differentiate(d_dot_coeffs);

	std::vector<float> D{ cost_function::pol_evaluation(trajectory.coeff_d,T),cost_function::pol_evaluation(d_dot_coeffs,T) ,cost_function::pol_evaluation(d_ddot_coeffs,T) };
	std::vector<float> target = predictions[target_vehicle].state_in(T);

	for (int id = 0; id < target.size(); id++)
	{
		target[id] += delta[id];
	}
	std::vector<float> d_target{ target[3],target[4],target[5] };

	float cost = 0.0f;
	for (int id = 0; id < D.size(); id++)
	{
		float actual = D[id];
		float expected = d_target[id];
		float sigma = sigma_d[id];
		float diff = fabsf(actual - expected);
		cost += logistic(diff / sigma);
	}
	return cost;
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
float stays_on_road_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	return 1;
}
//-----------------------------------------------------------------------------------
float exceeds_speed_limit_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	return 1;
}
//-----------------------------------------------------------------------------------
float efficiency_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{	
	//Rewards high average speeds.
	float avg_v = (float)(cost_function::pol_evaluation(trajectory.coeff_s, trajectory.t)) / trajectory.t;
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
		float acc = cost_function::pol_evaluation(s_d_dot, t);// a(t);
		total_acc += abs(acc*dt);
	}
	float acc_per_second = total_acc / T;
	return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC);
}
//-----------------------------------------------------------------------------------
float max_accel_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	std::vector<float> s_dot = differentiate(trajectory.coeff_s);
	std::vector<float> s_d_dot = differentiate(s_dot);
	std::vector<float> all_accs;
	float max_abs = 0.0f;

	for (int i = 0; i<100;i++)
	{
		//all_accs.push_back(pol_evaluation(s_dot, (float(T) / 100 * i)));
		float value = cost_function::pol_evaluation(s_d_dot, (float(T) / 100 * i));

		if (fabsf(max_abs) < fabsf(value)) 
		{
			max_abs = value;
		}
	}

	if (fabsf(max_abs) > MAX_ACCEL)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//-----------------------------------------------------------------------------------
float max_jerk_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	std::vector<float> s_dot = differentiate(trajectory.coeff_s);
	std::vector<float> s_d_dot = differentiate(s_dot);
	std::vector<float> jerk = differentiate(s_d_dot);

	float max_abs = 0.0f;
	for (int i = 0; i < 100; i++)
	{
		//all_accs.push_back(pol_evaluation(s_dot, (float(T) / 100 * i)));
		float value = cost_function::pol_evaluation(jerk, (float(T) / 100 * i));

		if (fabsf(max_abs) < fabsf(value))
		{
			max_abs = value;
		}
	}

	if (fabsf(max_abs) > MAX_JERK)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
//-----------------------------------------------------------------------------------
float total_jerk_cost_priv(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	std::vector<float> s_dot = differentiate(trajectory.coeff_s);
	std::vector<float> s_d_dot = differentiate(s_dot);
	std::vector<float> diff_s_d_dot = differentiate(s_d_dot);

	float total_jerk = 0.0f;
	float dt = T / 100.0f;
	float j = 0.0f;

	for (int i = 0; i < 100; i++)
	{
		float t = dt * (float)i;		 
		j = cost_function::pol_evaluation(diff_s_d_dot, t);
		total_jerk += fabsf(j*dt);
	}

	float jerk_per_second = total_jerk / T;

	return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC);
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
cost_function::cost_function()
{
	// Add functions cost 
	functions_map["time_diff_cost_priv"] = time_diff_cost_priv;
	functions_map["s_diff_cost_priv"] = s_diff_cost_priv;
	functions_map["d_diff_cost_priv"] = d_diff_cost_priv;
	functions_map["collision_cost_priv"] = collision_cost_priv;
	functions_map["buffer_cost_priv"] = buffer_cost_priv;
	functions_map["stays_on_road_cost_priv"] = stays_on_road_cost_priv;
	functions_map["exceeds_speed_limit_cost_priv"] = exceeds_speed_limit_cost_priv;
	functions_map["efficiency_cost_priv"] = efficiency_cost_priv;
	functions_map["total_accel_cost_priv"] = total_accel_cost_priv;
	functions_map["max_accel_cost_priv"] = max_accel_cost_priv;
	functions_map["max_jerk_cost_priv"] = max_jerk_cost_priv;
	functions_map["total_jerk_cost_priv"] = total_jerk_cost_priv;


	//// For testing 
	//trajectory_t trajectory;
	//int target_vehicle=0;
	//std::vector<float> delta{ 0, 0, 0, 0, 0, 0 };
	//float goal_t=5.0;
	//std::vector<vehicle> predictions;
	//vehicle veh;
	//std::vector<float> state{ 0, 10, 0, 0, 0, 0 };
	//veh.start_state = state;
	//predictions.push_back(veh);
	//std::vector<float> coeff_s{ 10.0, 10.0, 0.0, -3.7037037,1.85185185,-0.24691358 };
	//std::vector<float> coeff_d{ 4.0, 0.0, 0.0, -1.48148148, 0.74074074, -0.09876543 };

	//trajectory.coeff_s = coeff_s;
	//trajectory.coeff_d = coeff_d;
	//trajectory.t = 3.0f;

	//std::string cost_functions = "exceeds_speed_limit_cost_priv";
	//
	//int res = functions_map[cost_functions](trajectory, target_vehicle, delta, goal_t, predictions);
}
//-----------------------------------------------------------------------------------
float cost_function::estimate_all_costs(trajectory_t trajectory, int target_vehicle, std::vector<float> delta, float T, std::vector<vehicle> predictions)
{
	std::map<std::string, FnPtr>::iterator it = functions_map.begin();
	float cost = 0.0f;
	int cnt = 0;
	// Iterate over all the elements in the map
	while (it != functions_map.end())
	{
		// Get the function name
		std::string  function_name = it->first;
		
		// Execute the pertinent data
		cost += weight_cost[cnt]*functions_map[function_name](trajectory, target_vehicle, delta, T, predictions);
		it++;
		cnt++;
	}

	return cost;
}
//-----------------------------------------------------------------------------------
cost_function::~cost_function()
{
}
