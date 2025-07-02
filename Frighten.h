#pragma once
#include "ActorState.h"

class Frighten :
	public ActorState
{

public:
	Frighten(Ghost*, GhostState);
	~Frighten() {};
	void Update(const float &dt) override; //Update
public:
	static unsigned short int eatenNum; //number of eaten ghost to know which score texture is next
private:
	Ghost* ghost;
	GhostState previousState;
	unsigned short int animationCounter;//How many times will the white ghost texture flash before the state expires.
	float stateTime; //needed for this class
	static unsigned short int NumberOfGhostInFrightenMode;
	
private:
	void Animate(const float &stateTime,const float &dt);
	void Init() override;
	void Exit(const GhostState&) override; //Clean up
	

};

