#pragma once
#include "ActorState.h"


class Chase :
	public ActorState
{

public:
	Chase(Ghost*);
	~Chase() {};
	void Update(const float &) override; //Update
	
private:
	Ghost* ghost;
	uint16_t STATENUMBER; //how many times the ghost had been in this state
private:
	void Init() override;
	void Exit(const GhostState& state = eScatter) override; //Clean up
};

