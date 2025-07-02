#pragma once

#include "SFML/Graphics.hpp"
#include <iostream>
#include "LevelVariables.h"
#include "Ghost.h"

using namespace sf;

#ifndef H_ACTSTATE
#define H_ACTSTATE

enum GhostState { eGhostHouse, eScatter, eChase, eFrighten, eGameOver, eEaten, eTunneling }; //Possible states for a ghost, enum class should be used in the future!

class ActorState
{
public:
	
public:
	ActorState() { stateClock.restart().asSeconds(); }
	virtual ~ActorState() {  }
	virtual void Update(const float& dt) = 0; //Update state
	
protected:
	
	Clock stateClock;
private:
	virtual void Init() = 0; //Initialize variables
	virtual void Exit(const GhostState&) = 0; //preparing for exiting the state

};

#endif

