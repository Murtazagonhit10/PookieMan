
#include "Ghost.h"
#include <list>
#include <cstdlib>
#include <iostream>
#include "Map.h"

using namespace std;

Texture Ghost::ghostTexture;
Text Ghost::scoreText;
SoundBuffer Ghost::eaten_soundbuffer;
Sound Ghost::eaten_sound;


bool Ghost::collideWithPacman()
{
	sf::Vector2i ghostTempCoord = ghostTempCorrdinate();

	if (pacManTempCoordsOnLevel == ghostTempCoord)
		return true;
	return false;
}

Ghost::Ghost() : animation(this){

	if (!eaten_soundbuffer.loadFromFile("Sounds/eaten.wav")) {
		cout << "Failed to load eaten.wav" << endl;
	}
	eaten_sound.setBuffer(eaten_soundbuffer);
	isFrightened = false;
	frightenedAgain = false;
	previousTurningpoint = {0,0};
	visible = true;
	inTunnel = false;
	ghostTexture.loadFromFile("Textures/Ghost.png");
	ghostBody.setTexture(&ghostTexture);
	ChaseStateCounter = 0;
	ScatterStateCounter = 0;

	ghostBody.setSize(Vector2f(GHOSTBODYSIZE, GHOSTBODYSIZE));
	
	ghostBody.setOrigin(GHOSTBODYSIZE / 2, GHOSTBODYSIZE / 2);
	
	

	scoreText.setFont(font);
	scoreText.setFillColor(Color::Cyan);
	scoreText.setCharacterSize(27);
	scoreText.setOrigin(FRUITSIZE / 2, FRUITSIZE / 2);

	
}

Ghost::~Ghost()
{
	cout << "Ghost destructed" << endl;
	if (state)
		delete state;

}


bool Ghost::turningPointReached()
{

	Vector2i tempCoords = ghostTempCorrdinate();
	char c = Map::GetTile(tempCoords.x, tempCoords.y);

	if ((c == 'T' || c == 't' || c == 'O') && previousTurningpoint != tempCoords) {
	
		Vector2f tempPosition;
		tempPosition = { ghostBody.getPosition().x / CELLSIZE, ghostBody.getPosition().y / CELLSIZE };

		if (tempPosition.x >= tempCoords.x + TURNNZONELOWERBOUND && tempPosition.x <= tempCoords.x + TURNNZONEUPPERBOUND && tempPosition.y >= tempCoords.y + TURNNZONELOWERBOUND && tempPosition.y <= tempCoords.y + TURNNZONEUPPERBOUND) {
			previousTurningpoint = tempCoords;
			return true;
		}

	}
	return false;
}

bool Ghost::tunnelPointReached()
{
	Vector2i tempCoords = ghostTempCorrdinate();
	char c = Map::GetTile(tempCoords.x, tempCoords.y);

	if (c == 's') {
		inTunnel = true;
		return inTunnel;
	}

	return false;
}

void Ghost::calculateNewDirection()
{
	Vector2i tempCoordinates = ghostTempCorrdinate();
	float shortestDistFromTargetNode = 1000, temp;
	char c = Map::GetTile(tempCoordinates.x + 1, tempCoordinates.y);
	//Right
	if (direction.x != -1 && c != '#' && c != -1) {
		shortestDistFromTargetNode = eucledianDistance(tempCoordinates.x + 1, tempCoordinates.y, targetNode.x, targetNode.y);
		directionNode = { tempCoordinates.x + 1, tempCoordinates.y };

	}
	//Down
	c = Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1);
	if (direction.y != -1 && c != '#' && c != 'd' && c != '_' && c != -1) {
		temp = eucledianDistance(tempCoordinates.x, tempCoordinates.y + 1, targetNode.x, targetNode.y);
		if (temp <= shortestDistFromTargetNode) {
			shortestDistFromTargetNode = temp;
			directionNode = { tempCoordinates.x, tempCoordinates.y+1 };

		}
		
	}
	//Left
	c = Map::GetTile(tempCoordinates.x - 1, tempCoordinates.y);
	if (direction.x != 1 && c != '#' && c != -1) {
		temp = eucledianDistance(tempCoordinates.x - 1, tempCoordinates.y, targetNode.x, targetNode.y);
		if (temp <= shortestDistFromTargetNode) {
			shortestDistFromTargetNode = temp;
			directionNode = { tempCoordinates.x - 1, tempCoordinates.y };

		}
	}
	//Up
	c = Map::GetTile(tempCoordinates.x, tempCoordinates.y - 1);
	if (direction.y != 1 && c != '#' && c != 'd' && c != -1) {
		temp = eucledianDistance(tempCoordinates.x, tempCoordinates.y - 1, targetNode.x, targetNode.y);
		if (temp <= shortestDistFromTargetNode) {
			directionNode = { tempCoordinates.x, tempCoordinates.y - 1 };
		}

	}

	setDirection();
}

void Ghost::calculateNewDirectionEatenMode()
{
	Vector2i tempCoordinates = ghostTempCorrdinate();
	float shortestDistFromTargetNode = 1000, temp;

	//Right
	if (direction.x != -1 && Map::GetTile(tempCoordinates.x + 1, tempCoordinates.y) != '#') {
		shortestDistFromTargetNode = eucledianDistance(tempCoordinates.x + 1, tempCoordinates.y, targetNode.x, targetNode.y);
		directionNode = { tempCoordinates.x + 1, tempCoordinates.y };

	}

	//Up
	if (direction.y != 1 && Map::GetTile(tempCoordinates.x, tempCoordinates.y - 1) != '#') {
		temp = eucledianDistance(tempCoordinates.x, tempCoordinates.y - 1, targetNode.x, targetNode.y);
		if (temp < shortestDistFromTargetNode) {
			shortestDistFromTargetNode = temp;
			directionNode = { tempCoordinates.x, tempCoordinates.y - 1 };
		}

	}

	//Left
	if (direction.x != 1 && Map::GetTile(tempCoordinates.x - 1, tempCoordinates.y) != '#') {
		temp = eucledianDistance(tempCoordinates.x - 1, tempCoordinates.y, targetNode.x, targetNode.y);
		if (temp < shortestDistFromTargetNode) {
			shortestDistFromTargetNode = temp;
			directionNode = { tempCoordinates.x - 1, tempCoordinates.y };
		}
	}
	//Down
	if (direction.y != -1 && Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1) != '#') {
		temp = eucledianDistance(tempCoordinates.x, tempCoordinates.y + 1, targetNode.x, targetNode.y);
		if (temp < shortestDistFromTargetNode) {
			directionNode = { tempCoordinates.x, tempCoordinates.y + 1 };
		}

	}

	setDirection();
}

void Ghost::chooseRandomDirection()
{

	Vector2i tempCoordinates = ghostTempCorrdinate();

	list<Vector2i> possibleDirectionsList = { Vector2i{1,0},Vector2i{0,1},Vector2i{-1,0},Vector2i{0,-1} };


	//Right
	if (direction.x == -1 || Map::GetTile(tempCoordinates.x + 1, tempCoordinates.y) == '#' || direction.x == -1 || Map::GetTile(tempCoordinates.x + 1, tempCoordinates.y) == 's') {
		possibleDirectionsList.remove(Vector2i{ 1,0 });
	}
	//Down
	if (direction.y == -1 || Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1) == '#' || Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1) == 'd' || Map::GetTile(tempCoordinates.x, tempCoordinates.y + 1) == '_') {
		possibleDirectionsList.remove(Vector2i{ 0,1 });

	}
	//Left
	if (direction.x == 1 || Map::GetTile(tempCoordinates.x - 1, tempCoordinates.y) == '#' || direction.x == 1 || Map::GetTile(tempCoordinates.x - 1, tempCoordinates.y) == 's') {
		possibleDirectionsList.remove(Vector2i{ -1,0 });
	}
	//Up
	if (direction.y == 1 || Map::GetTile(tempCoordinates.x, tempCoordinates.y - 1) == '#' || Map::GetTile(tempCoordinates.x, tempCoordinates.y - 1) == 'd') {
		possibleDirectionsList.remove(Vector2i{ 0,-1 });
	}

	auto it = possibleDirectionsList.begin();

	advance(it, (rand() % possibleDirectionsList.size()));

	direction = *it;

}

void Ghost::playEatenSound()
{
	eaten_sound.play();

}

unsigned short int Ghost::getDirectionForAnimation() //Helps to select the column for the ghost
{
	unsigned short int col;
	
	if (currentState != 5) {
		direction.x == 1 ? col = 0
			: direction.x == -1 ? col = 2
			: direction.y == -1 ? col = 4
			: direction.y == 1 ? col = 6
			: col = 0;
	}
	else {

		direction.x == 1 ? col = 8
			: direction.x == -1 ? col = 9
			: direction.y == -1 ? col = 10
			: direction.y == 1 ? col = 11
			: col = 8;

	}

	return col;
}

Vector2f Ghost::ghostTempPosition() const
{

	return ghostBody.getPosition();
	
}

void Ghost::moveOn(const float& dt)
{

	Vector2f movement = {0.0f,0.0f};

	movement.x = speed * dt * direction.x;
	movement.y = speed * dt * direction.y;

	ghostBody.move(movement);
	return;
}
bool Ghost::tunnelTeleport()
{
	Vector2i tempCoords = ghostTempCorrdinate();
	char c = Map::GetTile(tempCoords.x, tempCoords.y);

	if (c == 'L' && direction.x == -1) {
		ghostBody.setPosition((27 * CELLSIZE) + 12, (18 * CELLSIZE) + 12);
		return true;
	}
	else if (c == 'R' && direction.x == 1) {
		ghostBody.setPosition((0 * CELLSIZE) + 12, (18 * CELLSIZE) + 12);
		return true;
	}
	return false;
}

void Ghost::UpdateTexture()
{
	ghostBody.setTextureRect(animation.uvRect);
}

void Ghost::setStartPosition()
{
	ghostBody.setPosition(Vector2f((startPoints.x * CELLSIZE) + (CELLSIZE / 2), MAPOFFSET + (startPoints.y * CELLSIZE) + (CELLSIZE / 2)));//
}

void Ghost::turnAround()
{
	direction.x *= -1;
	direction.y *= -1;
	previousTurningpoint = { 0,0 };
}

void Ghost::setDirection() /* where to turn */
{

	Vector2i ghostTempCoord = ghostTempCorrdinate();
	Vector2i newDirection;
	
	newDirection.x = ghostTempCoord.x - directionNode.x;
	newDirection.y = ghostTempCoord.y - directionNode.y;

	newDirection.x *= -1;
	newDirection.y *= -1;
	
	direction.x = newDirection.x;
	direction.y = newDirection.y;

	return;
}

void Ghost::setDirection(Vector2i direction_)
{
	direction = direction_;
}

bool Ghost::comeOutFromHouse()
{
	if (direction.y != -1) {

		direction.x = 0;
		direction.y = -1;

	}
	
	float posi = ghostBody.getPosition().y / CELLSIZE;

	if (15.60f < posi) { //Go to the middle first
		return false;
	}

	direction = {0,0};
	return true;
}

void Ghost::Draw(sf::RenderWindow& window)
{
	if(visible)
		window.draw(ghostBody);
	if (scoreText.getString() != "")
		window.draw(scoreText);

}

void Ghost::setTargetNode(const Vector2i& target)
{
	targetNode = target;
}


