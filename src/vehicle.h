#pragma once
#include <vector>
class vehicle
{

public:

	vehicle();
	vehicle(std::vector<float> start);
	std::vector<float> state_in(float dt);
	~vehicle();
	std::vector<float> start_state;
private: 

	//std::vector<float> start_state;

};

