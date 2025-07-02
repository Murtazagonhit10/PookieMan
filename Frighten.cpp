#include "Frighten.h"
#include "Scatter.h"
#include "Chase.h"
#include "Eaten.h"
#include "Tunneling.h"
#include "GhostGameOver.h"
#include "Map.h"

unsigned short int Frighten::eatenNum = 200;
unsigned short int Frighten::NumberOfGhostInFrightenMode = 0;

Frighten::Frighten(Ghost* ghostToHandle, GhostState prevState) {
	
	ghost = ghostToHandle;
	previousState = prevState;
	ghost->isFrightened = true;
	ghost->currentState = eFrighten;
	Init();
}


void Frighten::Update(const float& dt)
{
	float stateTime = Map::ClockFrightenMode.getElapsedTime().asSeconds();

	if (Game_Over || Game_Win) {
		Exit(eGameOver);
		return;
	}

	if (paused)
		return;

	if (ghost->frightenedAgain) {
		ghost->frightenedAgain = false;
		Init();
		return;
	}

	if (stateTime >= levelValues[LEVELNUMBER][13] || animationCounter >= levelValues[LEVELNUMBER][14]) {
		if (LEVELNUMBER > 3)
			Exit(eChase);
		else
			Exit(eChase);
		return;
	}

	
	if (ghost->collideWithPacman()) {
		paused = true;
		Map::pauseTime = 1;
		Ghost::scoreText.setPosition(ghost->ghostTempPosition());
		
		ghost->visible = false;

		eatenNum*=2;
		Ghost::scoreText.setString(std::to_string(eatenNum));
		Map::score += (eatenNum == 0 ? 200 : eatenNum == 1 ? 400 : eatenNum == 2 ? 800 : 1600);
		Map::LifeScoreCounter += (eatenNum == 0 ? 200 : eatenNum == 1 ? 400 : eatenNum == 2 ? 800 : 1600);

		if (eatenNum == 1600)
			eatenNum = 200;

		Exit(eEaten);
		return;
	}


	if (ghost->turningPointReached() || ghost->tunnelPointReached()) {
		if (ghost->inTunnel) {
			Exit(eTunneling);
			return;
		} 
	
		ghost->chooseRandomDirection();
	
	}

	Animate(stateTime, dt);
	ghost->UpdateTexture();

	ghost->moveOn(dt);
}

void Frighten::Animate(const float &stateTime,const float &dt)
{
	if (stateTime > levelValues[LEVELNUMBER][13] - 2) { //start to flash before 2 seconds of the state end
		if (ghost->ANIMATIONSWITCHTIME != 0.08) {
			ghost->ANIMATIONSWITCHTIME = 0.08;
			ghost->animation.lastImage = ghost->animation.firstImage + 3;
		}
		ghost->animation.UpdateFrightenAnimation(dt, ghost->ANIMATIONSWITCHTIME,animationCounter); //Increments animation counter for flashing
	}
	else {
		ghost->animation.Update(dt, ghost->ANIMATIONSWITCHTIME);
	}
	
}

void Frighten::Init()
{
	++NumberOfGhostInFrightenMode;
	Map::SetFirghtenSound();
	eatenNum = 100;
	ghost->speed = levelValues[LEVELNUMBER][12];
	ghost->turnAround();
	animationCounter = 0;
	stateClock.restart().asSeconds();

	ghost->animation.imageToSet.y = 0;
	ghost->animation.firstImage = 8;
	ghost->animation.imageToSet.x = ghost->animation.firstImage;
	ghost->animation.lastImage = ghost->animation.firstImage + 1;
	
}

void Frighten::Exit(const GhostState& state)
{
	ghost->isFrightened = false;

	ghost->ANIMATIONSWITCHTIME = 0.25;

	--NumberOfGhostInFrightenMode;
	if (NumberOfGhostInFrightenMode <= 0) {
		if (Map::EatenMode == ON)
			Map::SetHomeRunningBackGroundSound();
		else
			Map::SetBackGroundSound();
	}

	switch (state) {

		case eChase:
			ghost->turnAround();
			if (eatenNum > 0)
				eatenNum = 0;
			ghost->setState(new Chase(ghost));
			break;
		case eScatter:
			ghost->turnAround();
			if (eatenNum > 0)
				eatenNum = 0;
			ghost->setState(new Scatter(ghost));
			break;
		case eFrighten:
			ghost->turnAround();
			ghost->setState(new Frighten(ghost, state));
			break;
		case eEaten:
			ghost->setState(new Eaten(ghost, state));
			break;
		case eTunneling:
			ghost->setState(new Tunneling(ghost));
			break;
		case eGameOver:
			ghost->setState(new GhostGameOver(ghost));
			break;
	}
	
}