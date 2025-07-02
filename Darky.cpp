#include "Darky.h"
#include "GhostHouse.h"
#include "Luxy.h"
#include "Pacman.h"

Vector2i Darky::sTempCoordsOnLevel;

Darky::Darky()
{
	ghostHouseStartNode = { 13,18 };

	PriorityNumber = 1;

	startPoints.x = DARKYSTARTX;
	startPoints.y = DARKYSTARTY;

	SetStartState();
	SetStartParams();

}

Darky::~Darky()
{
	cout << "Darky destructor" << endl;
}

void Darky::Update(const float& dt)
{
	LuxyTempCoords = Luxy::sTempCoordsOnLevel;

	pacManTempCoordsOnLevel = Pacman::sTempCoordsOnLevel;
	PacmanTempDirection = Pacman::sTempDirectionOnLevel;

	state->Update(dt);
	if (sTempCoordsOnLevel == LuxyTempCoords)
		chaseLuxy = false;

	sTempCoordsOnLevel = getTempCoordsOnLevel();

}

Vector2i Darky::getPelletInFrontOfPacman() {

	Vector2i temp = pacManTempCoordsOnLevel + PacmanTempDirection;

	while (true) {
		char c;
		c = Map::GetTile(temp.x, temp.y);
		if (c == -1)
			break;
		else if (c == '.')
		{
			if (Map::isPelletActive(temp))
				return temp;
		}
		temp += PacmanTempDirection;
	}

	return LuxyTempCoords;

}

void Darky::setChaseTargetNode()
{
	
	if (manhattanDistance(getTempCoordsOnLevel().x, getTempCoordsOnLevel().y, LuxyTempCoords.x, LuxyTempCoords.y) < 3 && !chaseLuxy) {
		chaseLuxy = true;
		targetNode = LuxyTempCoords;
		return;
	}
	if (chaseLuxy) {
		targetNode = LuxyTempCoords;
		return;
	}
	targetNode = getPelletInFrontOfPacman();

	return;
}

bool Darky::moveToFourteenDotThirtyFive()
{
	
	

	return true;
}

void Darky::moveUpAndDown()
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

void Darky::calculateNewDirection()
{
	Ghost::calculateNewDirection();
}

void Darky::calculateDirectionTowardsLuxy()
{
	Vector2i tempCoordinates = ghostTempCorrdinate();
	float shortestDistFromTargetNode = 1000, temp;

	//Right
	if (Map::GetTile(tempCoordinates.x + 1, tempCoordinates.y) != '#') {
		shortestDistFromTargetNode = eucledianDistance(tempCoordinates.x + 1, tempCoordinates.y, targetNode.x, targetNode.y);
		directionNode = { tempCoordinates.x + 1, tempCoordinates.y };

	}
	//Down
	if (Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1) != '#' && Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1) != 'd' && Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1) != '_') {
		temp = eucledianDistance(tempCoordinates.x, tempCoordinates.y + 1, targetNode.x, targetNode.y);
		if (temp <= shortestDistFromTargetNode) {
			shortestDistFromTargetNode = temp;
			directionNode = { tempCoordinates.x, tempCoordinates.y + 1 };

		}

	}
	//Left
	if (Map::GetTile(tempCoordinates.x - 1, tempCoordinates.y) != '#') {
		temp = eucledianDistance(tempCoordinates.x - 1, tempCoordinates.y, targetNode.x, targetNode.y);
		if (temp <= shortestDistFromTargetNode) {
			shortestDistFromTargetNode = temp;
			directionNode = { tempCoordinates.x - 1, tempCoordinates.y };

		}
	}
	//Up
	if (Map::GetTile(tempCoordinates.x, tempCoordinates.y - 1) != '#' && Map::GetTile(tempCoordinates.x, tempCoordinates.y - 1) != 'd') {
		temp = eucledianDistance(tempCoordinates.x, tempCoordinates.y - 1, targetNode.x, targetNode.y);
		if (temp <= shortestDistFromTargetNode) {
			directionNode = { tempCoordinates.x, tempCoordinates.y - 1 };
		}

	}
	setDirection();
}

void Darky::setScatterTargetNode()
{
	targetNode = LuxyTempCoords;
}

void Darky::SetStartState()
{
	state = new GhostHouse(this);
}

void Darky::SetStartParams()
{

	chaseLuxy = false;
	rowForAnimation = 5;
	direction.y = -1;
	ActivateGhost = false;
	startDirection = { 0,-1 };

	currentState = GhostState::eGhostHouse;
	animation.firstImage = getDirectionForAnimation();
	animation.imageToSet.x = animation.firstImage;
	animation.imageToSet.y = rowForAnimation;

	animation.Update(0, ANIMATIONSWITCHTIME);
	setStartPosition();

	UpdateTexture();
}

Vector2i Darky::getTempCoordsOnLevel() const
{
	Vector2i Position;

	Position.x = (int)(ghostBody.getPosition().x / CELLSIZE);
	Position.y = (int)(ghostBody.getPosition().y / CELLSIZE);

	return Position;
}

unsigned short int Darky::GetActivationDotLimit()
{
	if (SpecialCounter)
		return 7;
	else
		return 0;
}
