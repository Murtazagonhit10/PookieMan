#pragma once
#include "ActorState.h"
class Eaten :
	public ActorState
{

public:
	Eaten(Ghost* ghostToHandle,const GhostState& prevState);
	virtual void Update(const float& dt);
private:
	Ghost* ghost;
	GhostState previousState;
	static unsigned short int NumberOfGhostInEatenMode;
private:
	void Init();
	void Exit(const GhostState&); //Clean up

};

