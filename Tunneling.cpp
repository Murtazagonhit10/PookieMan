#include "Tunneling.h"
#include "Chase.h"
#include "GhostGameOver.h"
#include "Map.h"
#include "Frighten.h"


Tunneling::Tunneling(Ghost* ghostToHandle) {

	ghost = ghostToHandle;
	ghost->currentState = eFrighten;
	Init();
}

void Tunneling::Update(const float& dt)
{

	float stateTime = stateClock.getElapsedTime().asSeconds();

	if (Game_Over || Game_Win) {
		Exit(eGameOver);
		return;
	}

	if (paused)
		return;

	//if eaten exit here
	if (ghost->collideWithPacman()) {
		Game_Over = true;
		paused = true;
		Exit(eGameOver);
		return;
	}
	
	ghost->animation.Update(dt, ghost->ANIMATIONSWITCHTIME);
	ghost->UpdateTexture();
	(this->*fToUpdate)(dt); //Function pointers FIGHTIN'!
	return; //Return immediatley
}

void Tunneling::TunnelingIn(const float& dt)
{
	if (ghost->tunnelTeleport()) {
		fToUpdate = &Tunneling::TunnelingOut;	
	}
	ghost->moveOn(dt);
}

void Tunneling::TunnelingOut(const float& dt)
{
	if (ghost->turningPointReached())
	{
		
		Exit(eChase);
		return;
	}
	ghost->moveOn(dt);
}

void Tunneling::Init()
{

	fToUpdate = &Tunneling::TunnelingIn;
	
	stateClock.restart().asSeconds();
	ghost->speed = levelValues[LEVELNUMBER][5];
	ghost->currentState = eTunneling;
	ghost->animation.firstImage = ghost->getDirectionForAnimation();
	ghost->animation.imageToSet.x = ghost->animation.firstImage;
	ghost->animation.imageToSet.y = ghost->rowForAnimation;
	ghost->animation.lastImage = ghost->animation.firstImage+1;
}


void Tunneling::Exit(const GhostState& state)
{
	ghost->inTunnel = false;
	switch (state) {
	case eChase:
		ghost->setState(new Chase(ghost));
		break;
	case eGameOver:
		ghost->setState(new GhostGameOver(ghost));
		break;
	}

}