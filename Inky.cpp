#include "Inky.h"
#include "Blinky.h"
#include "Scatter.h"
#include "GhostHouse.h"
#include "Pacman.h"

Inky::Inky(){
	
	startPoints.x = INKYSTARTX;
	startPoints.y = INKYSTARTY;
	ghostHouseStartNode = { 11,18 };
	PriorityNumber = 2;
	SetStartState();
}

Inky::~Inky()
{
	std::cout << "Inky destroyed";
}

void Inky::Update(const float& dt)
{
	/* update current positions to work with */
	pacManTempCoordsOnLevel = Pacman::sTempCoordsOnLevel;
	pacmanTempDirection = Pacman::sTempDirectionOnLevel;
	blinkyTempCoordsOnLevel = Blinky::sTempCoordsOnLevel;
	
	state->Update(dt); // Update actual state
	

}



void Inky::SetStartState()
{

	state = new GhostHouse(this);
}


void Inky::moveUpAndDown()
{
	//Starts up by default.

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

bool Inky::moveToFourteenDotThirtyFive()
{
	if (direction.x != 1) {
		direction.x = 1;
		direction.y = 0;
	}
	if (14.35 > (ghostBody.getPosition().x + 10.0f) / CELLSIZE) {
		return false;
	}

	return true;
}

void Inky::setChaseTargetNode()
{

	Vector2i temp;
	Vector2i vectorToBlinky;

	temp = pacManTempCoordsOnLevel;
	//If pacman is facing RIGHT
	if (pacmanTempDirection.x == 1)
		temp.x += 2;
	//If pacman is facing DOWN
	else if (pacmanTempDirection.y == 1)
		temp.y += 2;
	//If pacman is facing LEFT
	else if (pacmanTempDirection.x == -1)
		temp.x -= 2;
	//If pacman is facing UP
	else if (pacmanTempDirection.y == -1) {
		temp.x -= 2;
		temp.y -= 2;
	}

	vectorToBlinky = blinkyTempCoordsOnLevel - temp;

	vectorToBlinky *= -1;

	targetNode = temp + vectorToBlinky;

	return;
}

void Inky::setScatterTargetNode()
{
	targetNode = scatterTargetNode;
}

void Inky::SetStartParams()
{
	rowForAnimation = 2;
	direction.y = -1;
	
	startDirection = { 0,-1 };

	currentState = eGhostHouse;
	animation.firstImage = getDirectionForAnimation();
	animation.imageToSet.x = animation.firstImage;
	animation.imageToSet.y = rowForAnimation;

	animation.Update(0, ANIMATIONSWITCHTIME);
	setStartPosition();
	UpdateTexture();
	
}

unsigned short int Inky::GetActivationDotLimit()
{
	if (SpecialCounter)
		return 17;
	else
		return LEVELNUMBER == 0 ? 30 : 0;
	
}

