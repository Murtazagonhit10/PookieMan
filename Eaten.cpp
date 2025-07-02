#include "Eaten.h"
#include "Chase.h"
#include "Scatter.h"
#include "GhostHouse.h"
#include "Map.h"

unsigned short int Eaten::NumberOfGhostInEatenMode = 0;

Eaten::Eaten(Ghost* ghostToHandle,const GhostState& prevState)
{
	ghost = ghostToHandle;
	previousState = prevState;
	
	Init();
}

void Eaten::Update(const float& dt)
{
	float stateTime = stateClock.getElapsedTime().asSeconds();

	if (Game_Over || Game_Win) {
		Exit(eGameOver);
		return;
	}

	if (paused)
		return;

	if (ghost->ghostTempCorrdinate() == ghost->getTargetNode()) {
		ghost->setTargetNode(Vector2i(ghost->ghostHouseStartNode));
	}
	if (!ghost->visible)
		ghost->visible = true;

	if (ghost->ghostTempCorrdinate() == Vector2i(ghost->ghostHouseStartNode)) {
		Exit(eGhostHouse);
		return;
	}

	if (ghost->turningPointReached()) {

		ghost->calculateNewDirectionEatenMode();

		ghost->animation.imageToSet.x = ghost->getDirectionForAnimation();
	}
	ghost->animation.UpdateSingleImage();
	ghost->UpdateTexture();
	ghost->moveOn(dt);
	
}

void Eaten::Init()
{
	Map::EatenMode = ON;
	++NumberOfGhostInEatenMode;
	Map::SetHomeRunningBackGroundSound();
	ghost->playEatenSound();
	ghost->currentState = eEaten;
	ghost->speed = GHOSTBASICSPEED;
	ghost->animation.imageToSet.y = EYEBALLSTEXTUREROW;
	ghost->animation.imageToSet.x = ghost->getDirectionForAnimation();
	ghost->setTargetNode(Vector2i{ 13,18 });
	Map::GhostHousePriority.push(ghost->GetGhostPriorityNumber());
	stateClock.restart().asSeconds();
	
}

void Eaten::Exit(const GhostState& state)
{
	--NumberOfGhostInEatenMode;
	if (NumberOfGhostInEatenMode <= 0) {
		Map::EatenMode = OFF;
		if (Map::FrightenMode == ON)
			Map::SetFirghtenSound();
		else
			Map::SetBackGroundSound();
	}
		
	

	ghost->setSpeed(150); //Back to normal speed

	switch (state) {

	case eGhostHouse:
		
		ghost->setState(new GhostHouse(ghost));
		break;
	
	}
}
