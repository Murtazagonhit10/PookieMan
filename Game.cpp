#include "Game.h"
#include <fstream>
#include <iostream>
#include <exception>
#include <sstream>
#include <memory>

using std::cout;

Game::Game()
	: window(sf::VideoMode(680,900), "Pacman", sf::Style::Close | sf::Style::Titlebar ),
	dt(0.0)
{	
	window.setPosition(Vector2i(0,0));

	window.setFramerateLimit(60);
	gameState = new Menu();
	newStateToSet = NULL;
}

Game::~Game() {

	if(gameState) {
		delete gameState;
	}

}

void Game::run()
{
	
	while (window.isOpen())
	{

		newStateToSet = gameState->Update(window);
		
		if (newStateToSet) {
			delete gameState;
			gameState = newStateToSet;
			newStateToSet = NULL;
		
		}

	}

}
