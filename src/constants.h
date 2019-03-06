#pragma once
#include <vector>

#define N_SAMPLES (100U)

#define MAX_JERK (10.0f)// # m/s/s/s
#define MAX_ACCEL (10.0f)// # m/s/s

#define EXPECTED_JERK_IN_ONE_SEC (2)// # m/s/s
#define EXPECTED_ACC_IN_ONE_SEC  (1) //# m/s

#define SPEED_LIMIT (30.0f)
#define VEHICLE_RADIUS  (1.5f) //# model vehicle as circle to simplify collision detection

#define TIMESTEP (0.5f)

#define SIGMA_T (2.0f)

const float sigma_s[3]{ 20.0f, 10.0f, 5.1f };//s, s_dot, s_dot_dot
const float sigma_d[3]{ 2.0f, 3.0f, 3.0f };

typedef struct
{
	std::vector<float> goal_s;
	std::vector<float> goal_d;
	float t;
}goal_t;

typedef struct
{
	std::vector<float> coeff_s;
	std::vector<float> coeff_d;
	float t;
}trajectory_t;