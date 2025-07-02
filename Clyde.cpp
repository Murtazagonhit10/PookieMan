#include "Clyde.h"
#include "Scatter.h"
#include "GhostHouse.h"
#include "Pacman.h"

//#include "Chase.h"

Clyde::Clyde() {

	ghostHouseStartNode = { 16,18 };
	rowForAnimation = 3;

	startPoints.x = CLYDESTARTX;
	startPoints.y = CLYDESTARTY;

	PriorityNumber = 3;
	SetStartState();

}

Clyde::~Clyde()
{
	std::cout << "Clyde destroyed";
}

void Clyde::Update(const float& dt)
{

	pacManTempCoordsOnLevel = Pacman::sTempCoordsOnLevel;
	
	state->Update(dt);
	//moveOn(dt);
	//active = false;
	
	//handleState();
	//if (stateToSet) { /* Handling new states here, this is ... wierd? Try change it. */
	//	//delete state;
	//	delete stateStack.top();

	//	stateStack.pop();
	//	stateStack.push(stateToSet);

	//	//state = stateToSet;
	//	stateToSet = NULL;
	//}
	//targetMark.setPosition((targetNode.x * CELLSIZE) + CELLSIZE / 2, (targetNode.y * CELLSIZE) + CELLSIZE / 2);//Used to show where the target tile is atm
	//clydeCircle.setPosition(ghostBody.getPosition());
}

//void Clyde::Draw(RenderWindow& window)
//{
//
//	window.draw(ghostBody);
//	window.draw(targetMark);
//	//window.draw(clydeCircle);
//	//window.draw(realBody); //only for debug DELETE IT
//}


void Clyde::moveUpAndDown()
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

bool Clyde::moveToFourteenDotThirtyFive()
{
	if (direction.x != -1) {

		direction.x = -1;
		direction.y = 0;
	}
	//ghostBody.setPosition(ghostBody.getPosition());
	if (13.75f < (ghostBody.getPosition().x - 10.0f) / CELLSIZE) {
		return false;
	}

	return true;
}


void Clyde::setChaseTargetNode()
{
	if (manhattanDistance(getTempCoordsOnLevel().x, getTempCoordsOnLevel().y, pacManTempCoordsOnLevel.x, pacManTempCoordsOnLevel.y) <= 8)
		targetNode = scatterTargetNode;
	else
		targetNode = pacManTempCoordsOnLevel;
	return;
}

void Clyde::setScatterTargetNode()
{
	targetNode = scatterTargetNode;
}

void Clyde::SetStartState()
{
	state = new GhostHouse(this);
}

void Clyde::SetStartParams()
{
	rowForAnimation = 3;
	direction.y = -1;
	startDirection = { 0,-1 };
	setStartPosition();

	currentState = eGhostHouse;
	animation.firstImage = getDirectionForAnimation();
	animation.imageToSet.x = animation.firstImage;
	animation.imageToSet.y = rowForAnimation;

	animation.Update(0, ANIMATIONSWITCHTIME);
	UpdateTexture();
	
}


Vector2i Clyde::getTempCoordsOnLevel()
{
	Vector2i Position;

	Position.x = (int)(ghostBody.getPosition().x / CELLSIZE);
	Position.y = (int)(ghostBody.getPosition().y / CELLSIZE);

	return Position;
}

unsigned short int Clyde::GetActivationDotLimit()
{
	if (SpecialCounter) {
		if (GlobalCounter >= 32)
			SpecialCounter = false;
		return 50;
	}
	else
		return LEVELNUMBER == 0 ? 60 : 50;
}