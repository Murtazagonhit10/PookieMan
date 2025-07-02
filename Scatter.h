#pragma once
#include "ActorState.h"
#include "Ghost.h"

class Scatter :
	public ActorState
{

public:
	Scatter(Ghost*);
	~Scatter() {};
	void Update(const float &dt) override; //Update
private:
	Ghost* ghost;
	uint16_t STATENUMBER; //how many times the ghost had been in this state
private:
	void Init() override;
	
	void Exit(const GhostState& state = eChase) override; //Clean up
};

