#include "GhostHouse.h"
#include "Scatter.h"
#include "Frighten.h"
#include "Chase.h"
#include "Map.h"

GhostHouse::GhostHouse(Ghost* p_ghost)
{
	
	ghost = p_ghost;
	Init();
}

void GhostHouse::Update(const float &dt)
{

	if (Game_Over || Game_Win) {
		Exit(eGameOver);
		return;
	}

	if (paused)
		return;

	if (ghost->isFrightened && Map::ClockFrightenMode.getElapsedTime().asSeconds() >= levelValues[LEVELNUMBER][13]) {
		ghost->isFrightened = false;
		ghost->ANIMATIONSWITCHTIME = 0.25;
		SetUpAnimation();
	}

	if (ghost->isFrightened) { /* Setup Frighten mode animation, and frighten state variables */
		ghost->animation.imageToSet.y = 0;
		ghost->animation.firstImage = 8;
		ghost->animation.imageToSet.x = ghost->animation.firstImage;
		ghost->animation.lastImage = ghost->animation.firstImage + 1;

	}

	if (!ghost->ActivateGhost && (!Map::GhostHousePriority.empty() && ghost->GetGhostPriorityNumber() == Map::GhostHousePriority.top())) { //am I topPriority
		
		if (!DotCounterActive) {
			Map::DotCounterForGhosts = &DotCounter; //Turn on counting dots in map
			DotCounterActive = true;
		}

		if (DotCounter >= ghost->GetActivationDotLimit() || Map::ClockSinceLastDotEaten.getElapsedTime().asSeconds() >= (LEVELNUMBER >= 5 ? 3 : 4)) {//

			ghost->ActivateGhost = true;
			Map::GhostHousePriority.pop();
		}
	}

	if (ghost->ActivateGhost) {
		
		if (ghost->moveToFourteenDotThirtyFive()) {
			
			if (ghost->comeOutFromHouse()) {
				Exit();
				return;
			}
		}
		
	}
	else {
		ghost->moveUpAndDown();
	}

	Animate(dt);
	
	ghost->UpdateTexture();
	ghost->moveOn(dt);
}

void GhostHouse::Init()
{
	ghost->currentState = eGhostHouse;
	
	animationCounter = 0;

	ghost->ActivateGhost = false;
	DotCounterActive = false;
	DotCounter = 0;

	ghost->speed = levelValues[LEVELNUMBER][5];
	ghost->setDirection(ghost->startDirection);
	stateClock.restart().asSeconds();
	SetUpAnimation();
	
}

void GhostHouse::Exit(const GhostState& state){

	ghost->calculateNewDirection();
	
	if (ghost->isFrightened)
		ghost->setState(new Frighten(ghost,eScatter));
	else if(LEVELNUMBER < 3)
		ghost->setState(new Scatter(ghost));
	else
		ghost->setState(new Chase(ghost));
	return;
}


void GhostHouse::SetUpAnimation()
{
	
	ghost->animation.selectBox = { 16,16 }; //default 16x16 for ghosts
	ghost->animation.uvRect.width = 14;
	ghost->animation.uvRect.height = 14;
	ghost->animation.imageToSet.y = ghost->rowForAnimation;
	prevDirection = ghost->getDirectionForAnimation();
	ghost->animation.firstImage = prevDirection;
	ghost->animation.imageToSet.x = ghost->animation.firstImage;
	ghost->animation.lastImage = ghost->animation.firstImage + 1;
	
}

void GhostHouse::Animate(const float& dt)
{
	if (ghost->isFrightened) {
		float stateTime = Map::ClockFrightenMode.getElapsedTime().asSeconds();

		if (stateTime > levelValues[LEVELNUMBER][13]-2) { //start to flash before 2 seconds of the state end
			if (ghost->ANIMATIONSWITCHTIME != 0.08)
				ghost->ANIMATIONSWITCHTIME = 0.08;
			ghost->animation.lastImage = ghost->animation.firstImage + 3;
			ghost->animation.UpdateFrightenAnimation(dt, ghost->ANIMATIONSWITCHTIME, animationCounter); //Increments animation counter
		}
		else {
			ghost->animation.Update(dt, ghost->ANIMATIONSWITCHTIME);
		}
	}
	else {
		if (prevDirection != ghost->getDirectionForAnimation()) {
			prevDirection = ghost->getDirectionForAnimation();
			ghost->animation.firstImage = prevDirection;
			ghost->animation.imageToSet.x = ghost->animation.firstImage;
			ghost->animation.lastImage = ghost->animation.firstImage + 1;
		}
		ghost->animation.Update(dt, ghost->ANIMATIONSWITCHTIME);
	}
}
