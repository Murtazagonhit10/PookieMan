#include "Luxy.h"
#include "Scatter.h"
#include "Chase.h"
#include "Darky.h"
#include "GhostHouse.h"
#include "Pacman.h"

Vector2i Luxy::sTempCoordsOnLevel;

Luxy::Luxy() 
{
	ghostHouseStartNode = { 13,18 };

	startPoints.x = LUXYSTARTX;
	startPoints.y = LUXYSTARTY;

	PriorityNumber = 0;
	SetStartState();
	SetStartParams();
	chaseDarky = false;
}

Luxy::~Luxy()
{
	cout << "Luxy destructor" << endl;
}

void Luxy::Update(const float& dt)
{

	state->Update(dt);

	pacManTempCoordsOnLevel = Pacman::sTempCoordsOnLevel;
	PacmanTempDirection = Pacman::sTempDirectionOnLevel;
	DarkyTempCoords = Darky::sTempCoordsOnLevel;

	if (sTempCoordsOnLevel == DarkyTempCoords)
		chaseDarky = false;

	sTempCoordsOnLevel = getTempCoordsOnLevel();

}


void Luxy::setChaseTargetNode()
{
	Vector2i tempCoordinates = ghostTempCorrdinate();
	
	//Darky current or the dark path

	if (manhattanDistance(getTempCoordsOnLevel().x, getTempCoordsOnLevel().y, DarkyTempCoords.x, DarkyTempCoords.y) < 3 && !chaseDarky) {
		chaseDarky = true;
		targetNode = DarkyTempCoords;
		return;
	}
	if (chaseDarky) {
		targetNode = DarkyTempCoords;
		return;
	}
		
	targetNode = pacManTempCoordsOnLevel + (PacmanTempDirection * -1);

	return;
}

void Luxy::setScatterTargetNode()
{
	targetNode = DarkyTempCoords;
}

void Luxy::SetStartState()
{
	if (LEVELNUMBER < 3)
		setState(new Scatter(this));
	else
		setState(new Chase(this));
}

void Luxy::SetStartParams()
{

	rowForAnimation = 4;

	startDirection = { 1,0 };

	direction = startDirection;

	setStartPosition();

	animation.selectBox = { 16,16 }; //default 16x16 for ghosts
	animation.uvRect.width = 14;
	animation.uvRect.height = 14;
	animation.firstImage = getDirectionForAnimation();
	animation.imageToSet.x = animation.firstImage;
	animation.imageToSet.y = rowForAnimation;
	animation.Update(0, ANIMATIONSWITCHTIME);
	UpdateTexture();
}

Vector2i Luxy::getTempCoordsOnLevel() const
{
	Vector2i Position;

	Position.x = (int)(ghostBody.getPosition().x / CELLSIZE);
	Position.y = (int)(ghostBody.getPosition().y / CELLSIZE);

	return Position;
}

unsigned short int Luxy::GetActivationDotLimit()
{
	return 0;
}

void Luxy::calculateNewDirection()
{
	Ghost::calculateNewDirection();
}

void Luxy::calculateDirectionTowardsDarky()
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
