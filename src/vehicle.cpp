#include "vehicle.h"


vehicle::vehicle()
{
}

vehicle::vehicle(std::vector<float> start)
{
	start_state = start;
}
std::vector<float> vehicle::state_in(float dt)
{
	
	std::vector<float> s{start_state[0],start_state[1],start_state[2]};
	std::vector<float> d{ start_state[3],start_state[4],start_state[5]};

	std::vector<float> state = { s[0] + (s[1] * dt) + s[2] * dt*dt / 2.0f,
								s[1] + s[2] * dt,
								s[2],
								d[0] + (d[1] * dt) + d[2] * dt*dt / 2.0f,
								d[1] + d[2] * dt,
								d[2]
	};
	
	return state;
}
vehicle::~vehicle()
{
}
