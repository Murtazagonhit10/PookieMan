#pragma once
#include "SFML\Graphics.hpp"

using namespace sf;

#define PATH_SPRITES "Textures/SpritesClean.png"
#define PATH_BACKGROUND "Textures/background.png"

Texture allTheSpritesTexture; //This is for the Animation class.
Texture backgroundTexture; //This is for the Game class.

bool loadFiles() {
	bool success = true; // Maybe I want more load from files here.

	/*if (!allTheSpritesTexture.loadFromFile(PATH_SPRITES))
		success = false;*/

	if(!backgroundTexture.loadFromFile(PATH_BACKGROUND))
		success = false;

	return success;
}
