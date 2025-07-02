#include "Pacman.h"
#include <Windows.h>
#undef byte
using std::to_string;
using namespace std;

Vector2i Pacman::sTempCoordsOnLevel = { 0 ,0 };
Vector2i Pacman::sTempDirectionOnLevel = { 0 ,0 };
bool Pacman::DotSpeed = true;
float Pacman::speed = levelValues[LEVELNUMBER][1];
RectangleShape Pacman::body;
unsigned short int  Pacman::health = 3;
Text Pacman::pacman_hp_text;

Pacman::Pacman() {

	row = 1;
	havebufferedmove = false;

	tempDirection = STARTDIRECTION;
	bufferedDirection = tempDirection;

	body.setSize(Vector2f(PACMANSIZEX, PACMANSIZEY));
	body.setOrigin(PACMANSIZEX / 2, PACMANSIZEY / 2);
	body.setPosition(Vector2f{ (PACMANSTARTPOSX * CELLSIZE) - 12.0f, (((PACMANSTARTPOSY * CELLSIZE) - 12.0f)) });

	if (!pacman_hp_texture.loadFromFile("Textures/pacman.png"))
		cout << "could not load pacmanTexture file for hp";
	pacman_hp_sprite.setTexture(pacman_hp_texture);

	animation = new Animation(this);
	
	pacman_hp_sprite.setScale(3,3);
	pacman_hp_sprite.setTextureRect(IntRect(14, 0, 14, 14));
	pacman_hp_sprite.setPosition(1 * CELLSIZE, 35 * CELLSIZE+12);

	body.setTexture(&pacman_hp_texture);
	
	animation->Update(row, 0, ANIMATIONSWITCHTIME);
	body.setTextureRect(animation->uvRect);

	pacman_hp_text.setFillColor(Color::Yellow);
	pacman_hp_text.setFont(font);
	pacman_hp_text.setCharacterSize(37);
	pacman_hp_text.setPosition(1 * CELLSIZE+40, 35 * CELLSIZE+5);
	pacman_hp_text.setString("x3");
	health = 3;
}

Pacman::~Pacman()
{

	delete animation;
}

void Pacman::Draw(RenderWindow& window)
{

	window.draw(body);
	window.draw(pacman_hp_text);
	window.draw(pacman_hp_sprite);
}

bool Pacman::Update(const float& dTime)
{
	if (paused)
		return false;

	if (Keyboard::isKeyPressed(Keyboard::Up)) { //Should not be used like this. while(window.pollevent()) should be used, but since no other event used than keypressed I use it like this

		bufferedDirection.y = -1;
		bufferedDirection.x = 0;
		havebufferedmove = true;
	}
	else if (Keyboard::isKeyPressed(Keyboard::Down)) {
		bufferedDirection.y = 1;
		bufferedDirection.x = 0;
		havebufferedmove = true;
	}
	else if (Keyboard::isKeyPressed(Keyboard::Left)) {


		bufferedDirection.x = -1;
		bufferedDirection.y = 0;
		havebufferedmove = true;
	}
	else if (Keyboard::isKeyPressed(Keyboard::Right)) {

		bufferedDirection.x = 1;
		bufferedDirection.y = 0;

		havebufferedmove = true;
	}

	Vector2i tempCoords = getTempCoordsOnLevel();
	char c = Map::GetTile(tempCoords.x, tempCoords.y);

	if (havebufferedmove && c == 'T' || c == 't' || c == 'O') { //T,t,O are marked positions on the Map. These are the places where changing direction is allowed - except when turning around 180°.
		//In the buffered direction I check that: 
		//1, if the Tile in the buffered direction is blocked
		//2, is pacmans origin in the turning zone. The turning zone is like a little square inside the tile, it helps to tell if pacman can turn, prevents half of pacmans texture turn into the wall. The origin has to be inbetween the lower and upper bound, from all axis.
		Vector2f tempPosition;
		tempPosition = { body.getPosition().x / CELLSIZE, body.getPosition().y / CELLSIZE };

		if (bufferedDirection.x == 1 && Map::GetTile(tempCoords.x + 1, tempCoords.y) != '#' && (tempPosition.x >= tempCoords.x + TURNNZONELOWERBOUND && tempPosition.x <= tempCoords.x + TURNNZONEUPPERBOUND && tempPosition.y >= tempCoords.y + TURNNZONELOWERBOUND && tempPosition.y <= tempCoords.y + TURNNZONEUPPERBOUND)) { //Right
			tempDirection = bufferedDirection;
			row = 0;
		}
		else if (bufferedDirection.y == 1 && Map::GetTile(tempCoords.x, tempCoords.y + 1) != '#' && (tempPosition.x >= tempCoords.x + TURNNZONELOWERBOUND && tempPosition.x <= tempCoords.x + TURNNZONEUPPERBOUND && tempPosition.y >= tempCoords.y + TURNNZONELOWERBOUND && tempPosition.y <= tempCoords.y + TURNNZONEUPPERBOUND)) {
			tempDirection = bufferedDirection;
			row = 3;
		}
		else if (bufferedDirection.x == -1 && Map::GetTile(tempCoords.x - 1, tempCoords.y) != '#' && (tempPosition.x >= tempCoords.x + TURNNZONELOWERBOUND && tempPosition.x <= tempCoords.x + TURNNZONEUPPERBOUND && tempPosition.y >= tempCoords.y + TURNNZONELOWERBOUND && tempPosition.y <= tempCoords.y + TURNNZONEUPPERBOUND)) {
			tempDirection = bufferedDirection;
			row = 1;
		}
		else if (bufferedDirection.y == -1 && Map::GetTile(tempCoords.x, tempCoords.y - 1) != '#' && (tempPosition.x >= tempCoords.x + TURNNZONELOWERBOUND && tempPosition.x <= tempCoords.x + TURNNZONEUPPERBOUND && tempPosition.y >= tempCoords.y + TURNNZONELOWERBOUND && tempPosition.y <= tempCoords.y + TURNNZONEUPPERBOUND)) {
			tempDirection = bufferedDirection;
			row = 2;
		}

	}
	else if (bufferedDirection == (tempDirection * -1)) { /* Turning to the opposite direction is allowed anytime. */
		tempDirection = bufferedDirection;
		row = rowToSetForAnimation();
	}

	if (checkCollision(dTime)) { //if checkcollision is OK then pacman can move

		SetUpSpeed();

		body.move(Vector2f(speed * dTime * tempDirection.x, speed * dTime * tempDirection.y));
		
		animation->Update(row, dTime, ANIMATIONSWITCHTIME);
		body.setTextureRect(animation->uvRect);
	}

	tunnelTeleport(); //Just change position if pacman is @ the end of the tunnel

	/* Pacman should update his data, so the other Ghosts can work with them trough Map Class. */

	sTempCoordsOnLevel = getTempCoordsOnLevel();
	sTempDirectionOnLevel = tempDirection;



	return false;
}

Vector2i Pacman::getTempCoordsOnLevel() const /* Gives back the coordinates (1,1), (2,1) etc.. */
{
	Vector2i Position;

	Position.x = (int)(body.getPosition().x / CELLSIZE);
	Position.y = (int)(body.getPosition().y / CELLSIZE);

	return Position;
}

void Pacman::InitDeathAnimation()
{
	animation->AimageCount = Vector2u(12, 1);
	animation->uvRect.width = 16.0f;
	animation->uvRect.height = 14.0f;
}

bool Pacman::UpdateDeathAnimation(const float& dt)
{
	if (animation->UpdateCustomOfColumns(4, 12, dt, 0.10f) >= 12) {

		return true;
	}
	body.setTextureRect(animation->uvRect);
	return false;
}

void Pacman::SetStartState()
{
	row = 1;
	tempDirection = STARTDIRECTION;
	bufferedDirection = tempDirection;

	animation->AimageCount = { 2, 4 };

	animation->uvRect.width = 14.0f;
	animation->uvRect.height = 14.0f;

	body.setPosition(Vector2f{ (PACMANSTARTPOSX * CELLSIZE) - 12.0f, (((PACMANSTARTPOSY * CELLSIZE) - 12.0f)) });

	sTempCoordsOnLevel = getTempCoordsOnLevel();

	animation->Update(row, 0, ANIMATIONSWITCHTIME);
	body.setTextureRect(animation->uvRect);

}

void Pacman::IncreaseHealth()
{
	++health;
	pacman_hp_text.setString("x"+std::to_string(health));
	
}

void Pacman::DecreaseHealth()
{
	if(health > 0)
		--health;
	
	pacman_hp_text.setString("x" + std::to_string(health));
}


bool Pacman::checkCollision(const float& dTime)
{

	Vector2f moveWith = { speed * dTime * tempDirection.x,speed * dTime * tempDirection.y };
	Vector2f tempPosition = body.getPosition();
	Vector2i nextPosition;

	if (tempDirection.x == 1) {
		moveWith.x += OFFSET;
	}
	else if (tempDirection.y == 1) {
		moveWith.y += OFFSET;
	}
	else if (tempDirection.x == -1) {
		moveWith.x -= OFFSET;
	}
	else if (tempDirection.y == -1) {
		moveWith.y -= OFFSET;
	}

	nextPosition.x = int((tempPosition.x + moveWith.x) / CELLSIZE);
	nextPosition.y = int((tempPosition.y + moveWith.y) / CELLSIZE);
	char c = Map::GetTile(nextPosition.x, nextPosition.y);
	if (c != '#' && c != '_' && c != -1)
		return true;
	return false;
}



unsigned short int Pacman::rowToSetForAnimation()
{
	unsigned short int row;

	tempDirection.x == 1 ? row = 0
		: tempDirection.x == -1 ? row = 1
		: tempDirection.y == -1 ? row = 2
		: tempDirection.y == 1 ? row = 3
		: row = 0;

	return row;
}

void Pacman::tunnelTeleport()
{

	if (getTempCoordsOnLevel() == Vector2i{ 0,18 } && tempDirection.x == -1) {
		body.setPosition((27 * CELLSIZE) + 12, (18 * CELLSIZE) + 12);
	}
	else if (getTempCoordsOnLevel() == Vector2i{ 27,18 } && tempDirection.x == 1) {
		body.setPosition((0 * CELLSIZE) + 12, (18 * CELLSIZE) + 12);
	}

}

void Pacman::SetUpSpeed() /* Turn back normal speed */
{
	auto actualtempcoords = getTempCoordsOnLevel();

	if (Map::FrightenMode == ON) {
		if (DotSpeed == ON && speed != levelValues[LEVELNUMBER][10] && sTempCoordsOnLevel != actualtempcoords) //if pacman stepped one tile further turn off dot speed
			speed = levelValues[LEVELNUMBER][10];
		return;
	}

	if (DotSpeed == ON && speed != levelValues[LEVELNUMBER][2] && sTempCoordsOnLevel != actualtempcoords) { //same here
		speed = levelValues[LEVELNUMBER][2];
		return;
	}

}