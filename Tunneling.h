#pragma once
#include "ActorState.h"

class Tunneling:public ActorState
{

public:

	Tunneling(Ghost* ghostToHandle);
	virtual void Update(const float& dt);
	
private:
	Ghost* ghost;
	void (Tunneling::*fToUpdate)(const float& dt); //I wanted to try out a function pointer for practicing. It is used seperate tunneling in and out.

private:

	void Init();
	void Exit(const GhostState&); //Clean up
	void TunnelingIn(const float& dt);
	void TunnelingOut(const float& dt);

};

