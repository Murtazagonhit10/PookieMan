#include "Pinky.h"
#include "Scatter.h"
#include "GhostHouse.h"
#include "Pacman.h"

Pinky::Pinky(const bool& classic) {


	ghostHouseStartNode = { 13,18 };

	PriorityNumber = 2;
	
	if (classic) {
		startPoints.x = PINKYSTARTXCLASS;
		startPoints.y = PINKYSTARTYCLASS;
	}
	else {
		startPoints.x = PINKYSTARTX;
		startPoints.y = PINKYSTARTY;
	}

	SetStartState();
}

Pinky::~Pinky()
{
	std::cout << "Pinky destroyed";
}

void Pinky::Update(const float& dt)
{

	pacmanTempDirection = Pacman::sTempDirectionOnLevel;
	pacManTempCoordsOnLevel = Pacman::sTempCoordsOnLevel;

	state->Update(dt);


}


void Pinky::SetStartState()
{
	state = new GhostHouse(this);
}


void Pinky::moveUpAndDown()
{

	float tempCoords;

	if (direction.y == -1) {
		tempCoords = ((ghostBody.getPosition().y - 10.0f) / CELLSIZE);
		if ((tempCoords) <= 17.50f
			) {
			turnAround();

		}
	}
	else if (direction.y == 1) {
		tempCoords = ((ghostBody.getPosition().y + 10.0f) / CELLSIZE);
		if ((tempCoords) >= 19.50f
			) {
			turnAround();

		}
	}

}

bool Pinky::moveToFourteenDotThirtyFive()
{
	Image asd;
	if (direction.x != 1) {
		direction.x = 1;
		direction.y = 0;
	}

	if (14.35 > (ghostBody.getPosition().x + 10.0f) / CELLSIZE) {
		return false;
	}

	return true;
}

void Pinky::setChaseTargetNode()
{
	targetNode = pacManTempCoordsOnLevel;
	//If pacman is facing RIGHT
	if (pacmanTempDirection.x == 1)
		targetNode.x += 4;
	//If pacman is facing DOWN
	else if (pacmanTempDirection.y == 1)
		targetNode.y+= 4;
	//If pacman is facing LEFT
	else if (pacmanTempDirection.x == -1)
		targetNode.x -= 4;
	//If pacman is facing UP
	else if (pacmanTempDirection.y == -1) {
		targetNode.x -= 4;
		targetNode.y -= 4;
	}
	return;
}

void Pinky::setScatterTargetNode()
{
	targetNode = scatterTargetNode;
}

void Pinky::SetStartParams()
{
	rowForAnimation = 1;
	startDirection = { 0,-1 };
	direction = startDirection;


	animation.selectBox = { 16,16 }; //default 16x16 for ghosts
	animation.uvRect.width = 14;
	animation.uvRect.height = 14;

	currentState = eGhostHouse;

	animation.firstImage = getDirectionForAnimation();
	animation.imageToSet.x = animation.firstImage;
	animation.imageToSet.y = rowForAnimation;
	animation.lastImage = animation.firstImage + 1;
	setStartPosition();
	animation.Update(0, ANIMATIONSWITCHTIME);
	UpdateTexture();
}

unsigned short int Pinky::GetActivationDotLimit()
{

	if (SpecialCounter)
		return 17;
	else
		return LEVELNUMBER == 0 ? 30 : 0;
}