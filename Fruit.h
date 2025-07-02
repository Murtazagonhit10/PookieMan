#pragma once
#include "SFML\Graphics.hpp"
#include "LevelVariables.h"

#ifndef H_FRUIT
#define H_FRUIT

#define FRUITX 337
#define FRUITY 515
#define FRUITSIZE 33

using namespace sf;

class Fruit
{
public:
	enum fruitState { Active, Eaten , NotActive};
	Fruit();
	void Update();
	void Draw(RenderWindow& window);
	void setFruitTexture(unsigned short int&);
	bool checkCollision(FloatRect);
	void eaten();
	void activate(unsigned short int);
	void activate();
	void deactivate();
	static bool isactive() { return active; }
	void SetPosition(Vector2f&);

private:
	
private:
	static bool active;
	fruitState state;
	Texture fruitTexture;
	RectangleShape fruitShape;
	unsigned short int fruitNum; //0-7 means the different fruits
	IntRect pictureSizes[8] = { {0,0,12,12},{12,0,11,12},{23,0,12,12},{35,0,12,12},{47,0,11,13},{58,0,11,12},{69,0,12,13},{81,0,7,13} };  //{left,top,with,height}
	Text scoreText;
	Clock fruitClock;
};

#endif