#include "Scatter.h"
#include "Chase.h"
#include "Frighten.h"
#include "GhostGameOver.h"
#include "Tunneling.h"

Scatter::Scatter(Ghost* ghostToHandle) {
	
	ghost = ghostToHandle;	
	
	Init();
}

void Scatter::Update(const float &dt)
{
	if (Game_Over || Game_Win) {
		Exit(eGameOver);
		return;
	}

	if (paused)
		return;

	float stateTime = stateClock.getElapsedTime().asSeconds();

	ghost->animation.Update(dt, ghost->ANIMATIONSWITCHTIME);
	ghost->UpdateTexture();

	/*Collision check*/
	if (!Game_Over && ghost->collideWithPacman())
	{
		Game_Over = true;
		paused = true;
		Exit(eGameOver);
		return;
	}

	if (ghost->isFrightened) {

		Exit(eFrighten);
		return;
	}

	if (LEVELNUMBER < 3 && stateTime > scatterTimings[LEVELNUMBER][ghost->ScatterStateCounter]) {
		Exit(eChase);
		return;
	}

	if(ghost->turningPointReached() || ghost->tunnelPointReached()){

		if (ghost->inTunnel) {
			Exit(eTunneling);
			return;
		}

		ghost->calculateNewDirection();
		ghost->animation.firstImage = ghost->getDirectionForAnimation();
		ghost->animation.imageToSet.x = ghost->animation.firstImage;
		ghost->animation.lastImage = ghost->animation.firstImage + 1;
	}

	ghost->moveOn(dt);
}

void Scatter::Init()
{
	ghost->currentState = eScatter;
	

	ghost->speed = elroy1 ? levelValues[LEVELNUMBER][7] : elroy2 ? levelValues[LEVELNUMBER][9] : levelValues[LEVELNUMBER][4];
	stateClock.restart().asSeconds();
	ghost->animation.selectBox = { 16,16 }; //default 16x16 for ghosts
	ghost->animation.uvRect.width = 14;
	ghost->animation.uvRect.height = 14;
	
	ghost->setScatterTargetNode();
	ghost->animation.firstImage = ghost->getDirectionForAnimation();
	ghost->animation.imageToSet.x = ghost->animation.firstImage;
	ghost->animation.imageToSet.y = ghost->rowForAnimation;
	ghost->animation.lastImage = ghost->animation.firstImage+1;

}

void Scatter::Exit(const GhostState& state)
{
	if (ghost->ScatterStateCounter < 4)
		++ghost->ScatterStateCounter;
	switch (state) {

	case eChase:
		
		ghost->turnAround();
		ghost->setState(new Chase(ghost));
		break;
	case eFrighten:
		
		ghost->setState(new Frighten(ghost, ghost->currentState));
		break;
	case eGameOver:
		ghost->setState(new GhostGameOver(ghost));
		break;
	case eTunneling:
		ghost->setState(new Tunneling(ghost));
		break;
	}
}

