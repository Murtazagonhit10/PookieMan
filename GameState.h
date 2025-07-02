#pragma once
#include "LevelVariables.h"
#include <iostream>

using namespace sf;


class GameState
{
public:
	GameState() { font.loadFromFile("Joystix.TTF"); }
	virtual ~GameState() { }
	virtual GameState* Update(RenderWindow&) = 0;

};

