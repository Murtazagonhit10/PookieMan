#pragma once
#include "GameState.h"
#include "Blinky.h"
#include "Pinky.h"
#include "Inky.h"
#include "Clyde.h"
#include "Pacman.h"
#include "Darky.h"
#include "Luxy.h"
#include "SFML\Audio.hpp"

#define HIGHSCOREFILENAME "highscores.txt"
#define PACMANTEXTUREPATH "Textures/pacman.png"

class Playing : public GameState
{

public:
	Playing(bool classic);
	~Playing();
	GameState* Update(RenderWindow&);
	void Render(RenderWindow& window);

public:
	Clock deltaClock;

	Sprite backgroundSprite;

	Event event;

	Texture backgroundTexture; //This is for the Game class.

	Map map; //the map
	
	Pacman pacman; //The hero

	/* Ghosts start */
	Ghost* one;
	Ghost* two;
	Ghost* three;
	Ghost* four;
	/* Ghosts end */

	/* Sounds start */

	SoundBuffer intro_soundbuffer;
	Sound intro_sound;
	
	/* Sounds end */

private:

	int Setnewhighscore();

	void Updatedt();
	void UpdateGameWinAnimation(const float& );
	
	void GhostsVisible(bool);

	
	void UpdateGamePlay();
	void UpdateGameWin();
	void UpdateGameWinAnimation();
	void UpdateGameOver();
	void UpdateGameStart();

	void UpdateHighScore();
	
	void SetNextFruitLevelTexture();
	void InitFruitTextures();

	int GetCurrentHighScore();

private:

	//This function pointer updates the different states, gamewin, gameover etc.
	void (Playing::* fToUpdate)(); //Reasons for using: none, I just wanted to try out function pointers.
	
	Text readyText;
	Text highScoreText;

	/* clocks using this to store how much time passed */
	float pauseTimePassed = 0;
	float TimePassedFromWinAnimationStart;
	float TimePassedFromGameStartAnimation;
	float TimePassedWinAnimation = 0;

	const float SwitchWinAnimationAfter = 0.3f;
	const float FinishWinAnimationAfter = 3;

	unsigned short int StartGameWinAnimationAfter = 2;

	unsigned short int FruitLevelNumber = 0; //Which cell to use in FruitLevelTextures array, there can be 8

	Fruit FruitLevelTextures[8];

	bool whiteorblue = false; //blue or white background flashing
	IntRect backgroundAnimation[2] = {IntRect(0,0,672,744),IntRect(694,0,672,744)};

	int currenthighscore;
	int score;

	//deltatime
	float dt;

	/* Textures */
	Texture ghostsTexture;
	Texture pacmanTexture;
	
	
	//Return this to game object, every Update function sets this at the end of the function.
	GameState* returnState;
	
};

