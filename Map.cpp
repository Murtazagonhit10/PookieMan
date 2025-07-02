#include "Map.h"
#include "Pacman.h"
#include "Fruit.h"
#include <iostream>

#define _SILENCE_CXX17_ITERATOR_BASE_CLASS_DEPRECATION_WARNING

using std::iterator;

string Map::level = "";

Map::pellet Map::pelletArray[pelletnum];

list<Map::powerPellet> Map::powerPelletList;

priority_queue<unsigned short int, std::vector<unsigned short int>, std::greater<unsigned short int>>Map::GhostHousePriority;

int Map::score = 0;
unsigned short int Map::LifeScoreCounter = 0;
unsigned short int* Map::DotCounterForGhosts = NULL;
unsigned short int Map::numOfPellets = 240;
unsigned short int Map::pauseTime = 2;

bool Map::FrightenMode = OFF;
bool Map::EatenMode = OFF;

Sound Map::background_sound;
Sound Map::homerun_sound;
Sound Map::frighten_sound;

SoundBuffer Map::frighten_soundbuffer;
SoundBuffer Map::background1_soundbuffer;
SoundBuffer Map::background2_soundbuffer;
SoundBuffer Map::background3_soundbuffer;
SoundBuffer Map::background4_soundbuffer;
SoundBuffer Map::background5_soundbuffer;
SoundBuffer Map::homerun_soundbuffer;

Clock Map::ClockSinceLastDotEaten;
Clock Map::ClockFrightenMode;

Map::Map() {

	powerPellet p1(7, 1);
	powerPellet p2(7, 26);
	powerPellet p3(27, 1);
	powerPellet p4(27, 26);

	powerPelletList.push_back(p1);
	powerPelletList.push_back(p2);
	powerPelletList.push_back(p3);
	powerPelletList.push_back(p4);

	CellSize = CELLSIZE;
	uCellSize = static_cast<unsigned int>(CellSize);
	EatenMode = false;

	LoadMap();
	LoadPellets();

	pauseTime = 2;
	PowerPelletBlinkingClock.restart().asSeconds();

	fruitsSpawned = 0;
	FRUITNUMBER = 0;

	scoreText.setFillColor(Color::White);
	scoreText.setFont(font);
	scoreText.setCharacterSize(23);
	scoreText.setPosition(2 * CELLSIZE, 2 * CELLSIZE);
	score = 0;

	ClockSinceLastDotEaten.restart().asSeconds();

	numOfPellets = pelletnum;

	/* Load Sounds */
	LoadSound(waka_soundbuffer, waka_sound, wakasoundpath);
	LoadSound(fruiteaten_soundbuffer, fruiteaten_sound, fruiteatensoundpath);
	LoadSound(background2_soundbuffer, background_sound, backgroundsoundpath2);
	LoadSound(background3_soundbuffer, background_sound, backgroundsoundpath3);
	LoadSound(background4_soundbuffer, background_sound, backgroundsoundpath4);
	LoadSound(background5_soundbuffer, background_sound, backgroundsoundpath5);
	LoadSound(background1_soundbuffer, background_sound, backgroundsoundpath1);
	LoadSound(frighten_soundbuffer, frighten_sound, frightensoundpath);
	LoadSound(pacmanhpup_soundbuffer, pacmanhpup_sound, pacmanhpupsoundpath);
	LoadSound(homerun_soundbuffer, homerun_sound, homerunsoundpath);

	waka_sound.setPitch(1.2); //to make it a bit faster

}
Map::~Map()
{
	StopSound(background_sound);

	std::cout << "map destroyed" << std::endl;
}

void Map::Update()
{

	if (Game_Win || Game_Over) {
		StopSound(background_sound);
		return;
	}

	PlaySound(background_sound);

	if (checkFruitPacmanCollision()) {
		fruit.eaten();
		score += levelValues[LEVELNUMBER][1];
		LifeScoreCounter += levelValues[LEVELNUMBER][1];
		PlaySound(fruiteaten_sound);
	}

	if (FrightenMode == ON && ClockFrightenMode.getElapsedTime().asSeconds() >= levelValues[LEVELNUMBER][13]) {
		FrightenMode = OFF;

		Pacman::speed = levelValues[LEVELNUMBER][2];
	}

	scoreText.setString(std::to_string(score));

	if (fruitsSpawned < 2 && LEVELNUMBER < 22 && fruitAppear[FRUITNUMBER] >= numOfPellets) {
		fruit.activate(levelValues[LEVELNUMBER][0]);
		++FRUITNUMBER;
		++fruitsSpawned;
	}
	fruit.Update();


	return;
}

void Map::Draw(RenderWindow& window)
{
	window.draw(scoreText);
	fruit.Draw(window);
	for (auto& a : pelletArray)
		if (a.isActive) {
			window.draw(a.pelletBody);
		}
	if (PowerPelletBlinkingClock.getElapsedTime().asSeconds() < 0.5) {
		for (auto& a : powerPelletList)
			window.draw(a.powerPelletBody);
	}
	else if (PowerPelletBlinkingClock.getElapsedTime().asSeconds() > 0.8)
		PowerPelletBlinkingClock.restart().asSeconds();
}

void Map::handlePellets(Ghost*& gh1, Ghost*& gh2, Ghost*& gh3, Ghost*& gh4) //This makes the fruit appear aswell
{

	Vector2i pelletTempCoords;

	for (auto& a : powerPelletList) {

		if (a.getCoordinates() == Pacman::sTempCoordsOnLevel) { //Pacman-pellet collision

			powerPelletList.remove(a);

			//Setup pacman normal and dot speed
			Pacman::DotSpeed = ON;
			Pacman::speed = levelValues[LEVELNUMBER][10];

			FrightenMode = ON;
			score += 50;
			LifeScoreCounter += 50;
			ClockFrightenMode.restart().asSeconds();

			gh1->SetFrightenMode();
			gh2->SetFrightenMode();
			gh3->SetFrightenMode();
			gh4->SetFrightenMode();

			break;
		}
	}

	for (auto& a : pelletArray) {

		if (a.isActive && a.getCoordinates() == Pacman::sTempCoordsOnLevel) {
			//set up pacman dot speed here aswell
			score += 10;
			LifeScoreCounter += 10;
			a.isActive = false;
			Pacman::DotSpeed = ON;
			Pacman::speed = FrightenMode == OFF ? levelValues[LEVELNUMBER][3] : levelValues[LEVELNUMBER][11];
			if (!GhostHousePriority.empty())
				++(*DotCounterForGhosts); //increment dot counter
			if (SpecialCounter)
				++GlobalCounter;
			ClockSinceLastDotEaten.restart().asSeconds();
			--numOfPellets;
			PlaySound(waka_sound);

			break;
		}
	}
	if (LifeScoreCounter >= 10000) {
		LifeScoreCounter = 0;
		Pacman::IncreaseHealth();
		PlaySound(pacmanhpup_sound);
	}

	if (numOfPellets <= 0) {
		Game_Win = true;
		paused = true;
		fruitsSpawned = 0;
		fruit.deactivate();
		return;
	}

	if (!elroy2 && !elroy1 && numOfPellets <= levelValues[LEVELNUMBER][6]) {
		std::cout << "elroy1 is true" << std::endl;
		elroy1 = true;
	}
	if (!elroy2 && numOfPellets <= levelValues[LEVELNUMBER][8]) {
		elroy1 = false;
		elroy2 = true;
		std::cout << "elroy2 is true" << std::endl;
	}
}

void Map::TurnOffPellets()
{

	for (auto& a : pelletArray) {
		a.isActive = false;
		numOfPellets = 0;
	}
}


void Map::TurnOnPellets()
{

	for (auto& a : pelletArray) {
		a.isActive = true;
	}

	powerPelletList.clear();

	powerPellet p1(7, 1);
	powerPellet p2(7, 26);
	powerPellet p3(27, 1);
	powerPellet p4(27, 26);

	powerPelletList.push_back(p2);
	powerPelletList.push_back(p1);
	powerPelletList.push_back(p3);
	powerPelletList.push_back(p4);

	numOfPellets = pelletnum;

}

int Map::GetTile(int x, int y)
{
	if (x < 0 || x > 27 || y > 34 || y < 0)
		return -1; //means error

	return level[y * MAPWIDTH + x];
}

bool Map::isPelletActive(const Vector2i& coord)
{

	for (auto a : pelletArray) {
		if (a.getCoordinates() == coord)
			return a.isActive;
	}
	list<powerPellet>::iterator it;
	for (it = powerPelletList.begin(); it != powerPelletList.end(); ++it) {
		if (it->getCoordinates() == coord)
			return true;
	}

	return false;
}

void Map::SetFirghtenSound()
{
	if (background_sound.getBuffer() != &frighten_soundbuffer) {
		StopSound(background_sound);
		background_sound.setBuffer(frighten_soundbuffer);
		PlaySound(background_sound);
	}

}

void Map::SetHomeRunningBackGroundSound()
{
	if (background_sound.getBuffer() != &homerun_soundbuffer) {
		StopSound(background_sound);
		background_sound.setBuffer(homerun_soundbuffer);
		PlaySound(background_sound);
	}
}

void Map::SetBackGroundSound()
{
	StopSound(background_sound);
	if (numOfPellets < 192) {
		if (background_sound.getBuffer() != &background2_soundbuffer)
			background_sound.setBuffer(background2_soundbuffer);
	}
	else if (numOfPellets < 134) {
		if (background_sound.getBuffer() != &background3_soundbuffer)
			background_sound.setBuffer(background3_soundbuffer);
	}
	else if (numOfPellets < 86) {
		if (background_sound.getBuffer() != &background4_soundbuffer)
			background_sound.setBuffer(background4_soundbuffer);
	}
	else if (numOfPellets < 38) {
		if (background_sound.getBuffer() != &background5_soundbuffer)
			background_sound.setBuffer(background5_soundbuffer);
	}
	else {
		if (background_sound.getBuffer() != &background1_soundbuffer)
			background_sound.setBuffer(background1_soundbuffer);
	}
	PlaySound(background_sound);
}

bool Map::checkFruitPacmanCollision()
{

	return fruit.checkCollision(Pacman::getTempPosOnLevel());
}

void Map::ClearGhostHousePriorityQueue()
{
	while (!GhostHousePriority.empty()) GhostHousePriority.pop();
}

void Map::PlaySound(Sound& sound)
{
	if (sound.getStatus() != Sound::Status::Playing)
		sound.play();
}

void Map::StopSound(Sound& sound)
{
	if (sound.getStatus() == Sound::Status::Playing)
		sound.stop();
}

void Map::LoadMap()
{
	//35*28

	level += "############################"; // plus row
	level += "############################"; // plus row
	level += "############################"; // plus row
	level += "############################"; // plus row
	level += "############################";
	level += "#T....T.....T##T.....T....T#";
	level += "#.####.#####.##.#####.####.#";
	level += "#O####.#####.##.#####.####O#";
	level += "#.####.#####.##.#####.####.#";
	level += "#T....T..T..T..T..T..T....T#";
	level += "#.####.##.########.##.####.#";
	level += "#.####.##.########.##.####.#";
	level += "#T....T##T..T##T..T##T....T#";
	level += "######.#####.##.#####.######";
	level += "######.#####d##d#####.######";
	level += "######.##t  tttt  t##.######";
	level += "######.## ###__### ##.######";
	level += "######.## #      # ##.######";
	level += "L    sT  t#  tt  #t  Ts    R";
	level += "######.## #      # ##.######";
	level += "######.## ######## ##.######";
	level += "######.##t        t##.######";
	level += "######.## ######## ##.######";
	level += "######.## ######## ##.######";
	level += "#T....T..T..T##T..T..T....T#";
	level += "#.####.#####.##.#####.####.#";
	level += "#.####.#####d##d#####.####.#";
	level += "#O.T##T..T..T  T..T..T##T.O#";
	level += "###.##.##.########.##.##.###";
	level += "###.##.##.########.##.##.###";
	level += "#T.T..T##T..T##T..T##T..T.T#";
	level += "#.##########.##.##########.#";
	level += "#.##########.##.##########.#";
	level += "#T..........T..T..........T#";
	level += "############################";

}

void Map::LoadPellets()
{

	//Idea is that I can directly index the level array and here I can set the pelletArray elements directly aswell by using the index variable to help counting.
	//I only increase index if I passed a pellet (power or not).
	//Really wanted to access pellets with indexing.

	int index = 0;
	for (int i = 5; i < MAPROWS; ++i) {
		for (int j = 1; j < MAPCOLUMNS - 1; ++j) {

			if (level[i * MAPWIDTH + j] == '.' || level[i * MAPWIDTH + j] == 'T') {
				pelletArray[index].isActive = true;
				pelletArray[index].pelletBody.setPosition((j * CELLSIZE) + 9, (i * CELLSIZE) + 9);
				pelletArray[index].setCoordinates(j, i);
				++index;
			}
		}

	}

	return;
}

void Map::LoadSound(SoundBuffer& soundbuffer, Sound& sound, const string& path)
{

	if (!soundbuffer.loadFromFile(path))
		std::cout << "Failed to load soundfile: " << path << std::endl;
	sound.setBuffer(soundbuffer);

}