#pragma once
#include "ActorState.h"
#include "Ghost.h"
class GhostHouse :
	public ActorState
{

public:
	GhostHouse(Ghost*);

	~GhostHouse() {};
	void Update(const float& dt) override; //Update
private:
	Ghost* ghost;
	unsigned short int DotCounter;
	unsigned short int animationCounter;//How many times will the ghost texture flash before the state expires.
	unsigned short int prevDirection;
	bool DotCounterActive;
private:
	void Init() override;
	void Exit(const GhostState& state = eScatter) override; //Ready to exit state
	void SetUpAnimation();
	void Animate(const float&);
};

