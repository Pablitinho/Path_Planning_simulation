#include "ptg.h"

#include <random>
#include "eigen/Dense"
#include <map>

using Eigen::MatrixXd;
using Eigen::VectorXd;

//-----------------------------------------------------------------------------------------------
ptg::ptg()
{
}
//-----------------------------------------------------------------------------------------------
ptg::ptg(std::vector<float> start)
{
 
}
std::vector<float> ptg::JMT(std::vector<float> start, std::vector<float> goal, float T)
{
	/**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
	MatrixXd A = MatrixXd(3, 3);
	A << T * T*T, T*T*T*T, T*T*T*T*T,
		3 * T*T, 4 * T*T*T, 5 * T*T*T*T,
		6 * T, 12 * T*T, 20 * T*T*T;

	MatrixXd B = MatrixXd(3, 1);
	B << goal[0] - (start[0] + start[1] * T + .5*start[2] * T*T),
		goal[1] - (start[1] + start[2] * T),
		goal[2] - start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai * B;

	std::vector <float> result = { start[0], start[1], 0.5f*start[2] };

	for (int i = 0; i < C.size(); ++i) {
		result.push_back(C.data()[i]);
	}

	return result;
}
//-----------------------------------------------------------------------------------------------
goal_t ptg::Perturb_Goal(std::vector<float> goal_s, std::vector<float> goal_d, float t)
{
	float mu=0.0f;
	float sigma = 0.0f;

	std::random_device rd;
	std::mt19937 e2(rd());
	goal_t goal;
	goal.goal_s = goal_s;
	goal.goal_d = goal_d;

	for (int id=0;id<goal_s.size();id++)
	{
		mu = goal_s[id];
		sigma = sigma_s[id];

		std::normal_distribution<> dist(mu, sigma);
		goal.goal_s[id]=dist(e2);
	}
	
	for (int id = 0; id < goal_d.size(); id++)
	{
		mu = goal_d[id];
		sigma = sigma_d[id];

		std::normal_distribution<> dist(mu, sigma);
		goal.goal_d[id] = dist(e2);
	}
	goal.t = t;

	return goal;
}
//-----------------------------------------------------------------------------------------------
std::vector<float> ptg::Find_Trajectory(std::vector<float> start_s, std::vector<float> start_d,
	int target_vehicle, std::vector<float> delta, float time,
	std::vector<vehicle> predictions) 
{
	std::vector<float> best;
	vehicle target = predictions[target_vehicle];

	//all_goals = []
	std::vector<goal_t> goals;

    //float timestep = 0.5f;
	float t = time - 4.0f * TIMESTEP;
	while (t <= time + 4 * TIMESTEP)
	{
		std::vector<float> target_state = target.state_in(t);

		for (int id = 0; id < target_state.size(); id++)
		{
			target_state[id] += delta[id];
		}

		std::vector<float> goal_s{ target_state[0],target_state[1],target_state[2] };
		std::vector<float> goal_d{ target_state[3],target_state[4],target_state[5] };
		
		goal_t goal;
		goal.goal_d = goal_d;
		goal.goal_s = goal_s;
		goal.t = t;

		goals.push_back(goal);

		for (int id_sample = 0; id_sample < N_SAMPLES; id_sample++)
		{
			goal_t perturbed_goal= this->Perturb_Goal(goal_s, goal_d,t);
			perturbed_goal.t = t;

			goals.push_back(perturbed_goal);
		}

		t += TIMESTEP;
	}
	std::vector<trajectory_t> trajectories;

	for (int id_goal = 0; id_goal < goals.size(); id_goal++)
	{
		trajectory_t trajectory;

		trajectory.coeff_s= JMT(start_s, goals[id_goal].goal_s, t);
		trajectory.coeff_d = JMT(start_d, goals[id_goal].goal_d, t);
		trajectory.t = goals[id_goal].t;

		trajectories.push_back(trajectory);
	}

	return best;
}
//-----------------------------------------------------------------------------------------------
ptg::~ptg()
{
}
//-----------------------------------------------------------------------------------------------