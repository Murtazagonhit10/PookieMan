#include "Chase.h"
#include "Scatter.h"
#include "Frighten.h"
#include "GhostGameOver.h"
#include "Tunneling.h"


Chase::Chase(Ghost * pghost) {
	
	ghost = pghost;
	//std::cout << "CHASE\n";
	Init();
}

void Chase::Update(const float& dt)
{

	if (Game_Over || Game_Win) {
		Exit(eGameOver);
		return;
	}

	if (paused)
		return;

	float stateTime = stateClock.getElapsedTime().asSeconds();

	if (!Game_Over && ghost->collideWithPacman())
	{
		paused = true;
		Game_Over = true;
		Exit(eGameOver);
		return;
	}

	ghost->animation.Update(dt,ghost->ANIMATIONSWITCHTIME);
	ghost->UpdateTexture();

	if (ghost->isFrightened) {
	
		Exit(eFrighten);
		return;
	}

	if (LEVELNUMBER < 3 && stateTime > chaseTimings[LEVELNUMBER][ghost->ScatterStateCounter]) {
		Exit();
		return;
	}

	if (ghost->turningPointReached() || ghost->tunnelPointReached()) {
		if (ghost->inTunnel) {
			Exit(eTunneling);
			return;
		}
		ghost->setChaseTargetNode();
		ghost->calculateNewDirection();
		ghost->animation.firstImage = ghost->getDirectionForAnimation();
		ghost->animation.imageToSet.x = ghost->animation.firstImage;
		ghost->animation.lastImage = ghost->animation.firstImage + 1;
	}
	ghost->moveOn(dt);
}

void Chase::Init()
{
	if (ghost->ChaseStateCounter<4)
		++ghost->ChaseStateCounter;
	ghost->currentState = eChase;
	
	ghost->speed = elroy1 ? levelValues[LEVELNUMBER][7] : elroy2 ? levelValues[LEVELNUMBER][9] : levelValues[LEVELNUMBER][4];

	stateClock.restart().asSeconds();
	//Flip direction
	
	ghost->turnAround();
	ghost->animation.firstImage = ghost->getDirectionForAnimation();
	ghost->animation.imageToSet.x = ghost->animation.firstImage;
	ghost->animation.imageToSet.y = ghost->rowForAnimation;
	ghost->animation.lastImage = 2;
}

void Chase::Exit(const GhostState& state)
{
	
	switch (state) {
		case eScatter:
			++STATENUMBER;
			ghost->turnAround();
			ghost->setState(new Scatter(ghost));
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
