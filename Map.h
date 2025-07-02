#include <SFML\Graphics.hpp>
#include <string>
#include <list>
#include <stack>
#include <queue>
#include <iostream>
#include "LevelVariables.h"
#include "Fruit.h"
#include "Ghost.h"
#include "SFML\Audio\Sound.hpp"
#include "SFML\Audio\SoundBuffer.hpp"

/* I consider Map as the bridge between playing class and the character classes pacman, ghosts etc. There are global variables*/

using namespace sf;
using std::string;
using std::list;
using std::stack;
using std::priority_queue;
using std::vector;

#ifndef MAP_H

#define MAP_H

extern bool paused;
extern bool elroy1;
extern bool elroy2;


#define MAPOFFSET 4*24 //Needed this for the start positions of the characters
#define MAPWIDTH 28
#define MAPROWS 34 //not used just to remember
#define MAPCOLUMNS 28 //not used just to remember
#define CELLSIZE 24
#define pelletnum 240

#define wakasoundpath "Sounds/wakawaka.wav"
#define frightensoundpath "Sounds/frighten.wav"
#define fruiteatensoundpath "Sounds/fruiteaten.wav"
#define backgroundsoundpath1 "Sounds/background1.wav"
#define backgroundsoundpath2 "Sounds/background2.wav"
#define backgroundsoundpath3 "Sounds/background3.wav"
#define backgroundsoundpath4 "Sounds/background4.wav"
#define backgroundsoundpath5 "Sounds/background5.wav"
#define pacmanhpupsoundpath "Sounds/lifeup.wav"
#define homerunsoundpath "Sounds/homerunning.wav"

class Map
{

public:

	Map();
	~Map();

	void Update();
	void Draw(RenderWindow& window);
	void handlePellets(Ghost*& gh1, Ghost*& gh2, Ghost*& gh3, Ghost*& gh4);
	void TurnOnPellets();
	void TurnOffPellets();

	static int GetTile(int x, int y);
	static bool isPelletActive(const Vector2i&);
	static void SetFirghtenSound();
	static void SetHomeRunningBackGroundSound();
	static void SetBackGroundSound();

	bool checkFruitPacmanCollision();

public:
	
	static int score;
	static unsigned short int LifeScoreCounter; //if >= 10000 increase pacman life
	static unsigned short int pauseTime; //how long should be the game paused, Frighten mode sets this
	unsigned short int FRUITNUMBER;
	void ClearGhostHousePriorityQueue();
	static unsigned short int* DotCounterForGhosts;
	static Clock ClockSinceLastDotEaten;
	static Clock ClockFrightenMode;
	static bool FrightenMode;
	static bool EatenMode;
	static priority_queue<unsigned short int,std::vector<unsigned short int>, std::greater<unsigned short int>> GhostHousePriority;
private:

	Text scoreText;
	
	Fruit fruit;

	unsigned short int fruitsSpawned;



	typedef struct pellet {

	private:

		Vector2i coordinates;
		
	public:

		bool isActive = false;
		RectangleShape pelletBody;

		pellet() {

			pelletBody.setSize(Vector2f(5, 5));
			pelletBody.setFillColor(Color(255, 183, 174));
			pelletBody.setOrigin(2, 2);

		}

	public:
		void setCoordinates(int x, int y) {
			coordinates = { x,y };
		}
		Vector2i getCoordinates() {
			return coordinates;
		}
	}pellet;

	/////////////////Power pellets

	typedef struct powerPellet {
	private:
		Vector2i coordinates;
	public:
		CircleShape powerPelletBody;

	public:
		powerPellet(int x, int y) {

			powerPelletBody.setRadius(8);
			powerPelletBody.setFillColor(Color(255, 183, 174));
			powerPelletBody.setOrigin(4, 4);
			powerPelletBody.setPosition((y * CELLSIZE) + 4, (x * CELLSIZE) + 4);
			coordinates = { y,x };
		}
		bool operator == (const powerPellet& s) const { return coordinates.x == s.coordinates.x && coordinates.y == s.coordinates.y; }

		Vector2i getCoordinates() {
			return coordinates;
		}
	}powerPellet;

	Clock PowerPelletBlinkingClock;

	static string level;
	float CellSize;
	unsigned int uCellSize;

	static unsigned short int numOfPellets;
	
	static pellet pelletArray[pelletnum];
	static list<powerPellet> powerPelletList; 

	SoundBuffer waka_soundbuffer;
	
	SoundBuffer fruiteaten_soundbuffer;
	SoundBuffer pacmanhpup_soundbuffer;

	static SoundBuffer background1_soundbuffer;
	static SoundBuffer background2_soundbuffer;
	static SoundBuffer background3_soundbuffer;
	static SoundBuffer background4_soundbuffer;
	static SoundBuffer background5_soundbuffer;

	static SoundBuffer frighten_soundbuffer;
	static SoundBuffer homerun_soundbuffer;

	Sound waka_sound;
	
	Sound fruiteaten_sound;
	Sound pacmanhpup_sound;

	static Sound homerun_sound;
	static Sound frighten_sound;
	static Sound background_sound;

private:
	static void PlaySound(Sound& sound);
	static void StopSound(Sound& sound);
	void LoadMap();
	void LoadPellets();
	void LoadSound(SoundBuffer& soundbuffer, Sound& sound, const string& path);
};

#endif