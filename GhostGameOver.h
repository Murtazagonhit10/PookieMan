#pragma once
#include "ActorState.h"
#include "Ghost.h"

class GhostGameOver :
	public ActorState
{
public:
	GhostGameOver(Ghost* ghostToHandle);
	virtual void Update(const float& dt);
private:
	Ghost* ghost;
private:
	void Init();
	void Exit(const GhostState&); //Clean up
};

