#include "Playing.h"
#include "Menu.h"
#include <fstream>
#include <iostream>

Playing::Playing(bool classic) {

	LEVELNUMBER = 0;

	if (!backgroundTexture.loadFromFile("Textures/background2.png"))
		std::cout << "could not load background file";

	backgroundSprite.setTexture(backgroundTexture);
	backgroundSprite.setPosition(0.0f, 4 * 24); //4*24 is the extra 4 row

	paused = false;

	readyText.setFillColor(Color::Yellow);
	readyText.setFont(font);
	readyText.setCharacterSize(37);
	readyText.setPosition(11 * CELLSIZE, 21 * CELLSIZE);
	readyText.setString("READY!");

	GetCurrentHighScore();

	highScoreText.setFillColor(Color::White);
	highScoreText.setFont(font);
	highScoreText.setCharacterSize(23);
	highScoreText.setPosition(9 * CELLSIZE, 0 * CELLSIZE);
	highScoreText.setString("HIGH SCORE: " +  std::to_string(currenthighscore));

	returnState = NULL;

	InitFruitTextures();
	SetNextFruitLevelTexture();
	
	/* These are fix */
	one = new Clyde();
	two = new Pinky(classic);

	if (classic) {
		three = new Blinky();
		four = new Inky();
	}
	else {
		three = new Luxy();
		four = new Darky();
	}

	one->SetStartParams();
	two->SetStartParams();
	three->SetStartParams();
	four->SetStartParams();

	SpecialCounter = false;
	
	intro_soundbuffer.loadFromFile("Sounds/intro.wav");
	intro_sound.setBuffer(intro_soundbuffer);

	fToUpdate = &Playing::UpdateGameStart;
}

Playing::~Playing()
{
	cout << "Playing destructed" << endl;
	delete one;
	delete two;
	delete three;
	delete four;
}

void Playing::Updatedt()
{
	dt = deltaClock.restart().asSeconds();
}

void Playing::UpdateGameWinAnimation(const float& dt)
{

	backgroundSprite.setTextureRect(backgroundAnimation[whiteorblue]);
	whiteorblue = !whiteorblue;
	StartGameWinAnimationAfter = 0;

}

int Playing::Setnewhighscore() {

	ofstream file(HIGHSCOREFILENAME);
	if (!file.is_open())
	{
		cout << "Could not open high scores file! Creating new one with the name: " << HIGHSCOREFILENAME << '\n';
		ofstream ostream(HIGHSCOREFILENAME);
		ostream << "";
		ostream.close();

		file.open(HIGHSCOREFILENAME);
		if (!file.is_open()) //if still cannot open
		{
			cout << "Unable to open " << HIGHSCOREFILENAME << endl;
			return 0;
		}
		
	}
	
	file << currenthighscore;

	if (file.fail())
		return 0;

	file.close();

	return 1;
}

void Playing::GhostsVisible(bool b)
{
	three->visible = b;
	four->visible = b;
	two->visible = b;
	one->visible = b;
}

void Playing::UpdateGamePlay()
{
	if (paused) {
		pauseTimePassed += dt; //wait for it.
		if (pauseTimePassed > map.pauseTime) { //and now setup the next mode, game win or game over
			paused = false;
			pauseTimePassed = 0;

			Ghost::scoreText.setString("");
			if (Game_Over) {
				GhostsVisible(false);
				pacman.InitDeathAnimation();
				fToUpdate = &Playing::UpdateGameOver;//func to update change here if gameover
				returnState = NULL;
				SpecialCounter = true;
				return;
			}
			else if (Game_Win) {
				GhostsVisible(false);
				fToUpdate = &Playing::UpdateGameWin;//func to update change here if win
				
				returnState = NULL;
				return;
			}
		}
	}

	map.Update();

	UpdateHighScore();

	pacman.Update(dt);

	map.handlePellets(one,two,three,four); //Handle pellets after pacman moved

	four->Update(dt);
	three->Update(dt);
	two->Update(dt);
	one->Update(dt);

	returnState = NULL;
	return;
}

void Playing::UpdateGameWin()
{

	TimePassedFromWinAnimationStart += dt;

	if (TimePassedFromWinAnimationStart < FinishWinAnimationAfter) {
		UpdateGameWinAnimation();
	}
	else {
		SpecialCounter = false;
		
		backgroundSprite.setTextureRect(backgroundAnimation[0]);

		GhostsVisible(true);

		readyText.setString("Ready!");

		map.TurnOnPellets(); //reload pellets
		
		if (LEVELNUMBER < 20) //next level
			++LEVELNUMBER;

		/* Set back start parameters */
		four->SetStartParams();
		three->SetStartParams();
		two->SetStartParams();
		one->SetStartParams();
		pacman.SetStartState();

		
		fToUpdate = &Playing::UpdateGameStart;

		TimePassedFromWinAnimationStart = 0;
		SetNextFruitLevelTexture();
		return;
	}

	returnState = NULL;
}

void Playing::UpdateGameWinAnimation()
{
	TimePassedWinAnimation += dt;

	if (TimePassedWinAnimation > SwitchWinAnimationAfter) {
		backgroundSprite.setTextureRect(backgroundAnimation[whiteorblue]);
		whiteorblue = !whiteorblue;
		StartGameWinAnimationAfter = 0;
		TimePassedWinAnimation = 0;
	}
}

void Playing::UpdateGameOver()
{

	if (pacman.UpdateDeathAnimation(dt)) {

		pacman.DecreaseHealth();
		if (pacman.GetHealthValue() <= 0) { //Game over go back to main menu
			returnState = new Menu();
			return;
		}
		else { 
			/* Set back start positions */
			pacman.SetStartState();
			three->SetStartParams();
			four->SetStartParams(); 
			two->SetStartParams();
			one->SetStartParams();
			readyText.setString("Ready!");
			GhostsVisible(true);
			fToUpdate = &Playing::UpdateGameStart;

			map.ClearGhostHousePriorityQueue();
			return;
		}
	}

	returnState = NULL;
}

void Playing::UpdateGameStart()
{

	TimePassedFromGameStartAnimation += dt;

	if (TimePassedFromGameStartAnimation > 4) {
		intro_sound.stop();
		TimePassedFromGameStartAnimation = 0;
		GhostsVisible(true);
		/* Reset all level variable here. */
		Game_Win = OFF;
		Game_Over = OFF;
		elroy1 = OFF;
		elroy2 = OFF;

		/* Set back start positions */
		pacman.SetStartState();
		three->SetStartState();
		four->SetStartState();
		two->SetStartState();
		one->SetStartState();
		map.ClearGhostHousePriorityQueue();
		map.GhostHousePriority.push(four->GetGhostPriorityNumber());
		map.GhostHousePriority.push(two->GetGhostPriorityNumber());
		map.GhostHousePriority.push(one->GetGhostPriorityNumber());
		fToUpdate = &Playing::UpdateGamePlay;
		readyText.setString("");
		map.ClockSinceLastDotEaten.restart();
		
		return;
	}
	
	if (intro_sound.getStatus() != Sound::Status::Playing)
		intro_sound.play();

	returnState = NULL;
}

void Playing::UpdateHighScore()
{
	if (map.score > currenthighscore) {
		currenthighscore = map.score;

		highScoreText.setString("HIGH SCORE: " + std::to_string(currenthighscore));
	}
}

void Playing::SetNextFruitLevelTexture()
{
	FruitLevelTextures[FruitLevelNumber].activate();
	FruitLevelTextures[FruitLevelNumber].setFruitTexture(levelValues[LEVELNUMBER][0]);
	++FruitLevelNumber;
	if (FruitLevelNumber > 7)
		FruitLevelNumber = 0;
}

void Playing::InitFruitTextures()
{
	Vector2f position = {300 ,36 * CELLSIZE + 5,};


	for (unsigned short int i = 0; i< 8 ; ++i,position.x+=40)
	{
		FruitLevelTextures[i].SetPosition(position);
		FruitLevelTextures[i].deactivate();
	}

}

int Playing::GetCurrentHighScore()
{
	ifstream file(HIGHSCOREFILENAME);

	if (!file.is_open())
	{
		cout << "Could not open high scores file! Creating new one with the name: " << HIGHSCOREFILENAME << '\n';
		ofstream ostream(HIGHSCOREFILENAME);
		ostream << "0";
		ostream.close();

		file.open(HIGHSCOREFILENAME);
		if (!file.is_open()) //if still cannot open ...
		{
			cout << "Unable to open " << HIGHSCOREFILENAME << endl;
			return 0;
		}

	}
	file >> currenthighscore;

	file.close();
}

void Playing::Render(RenderWindow& window) {

	window.clear();

	window.draw(backgroundSprite);

	map.Draw(window);
	four->Draw(window);
	three->Draw(window);
	pacman.Draw(window);
	two->Draw(window);
	one->Draw(window);

	if (readyText.getString() != "")
		window.draw(readyText);
	window.draw(highScoreText);
	for (auto& a : FruitLevelTextures)
		a.Draw(window);

	window.display();
}


GameState* Playing::Update(RenderWindow& window)
{
	while (window.pollEvent(event))
	{
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) {
			if (map.score >= currenthighscore) Setnewhighscore();
			return new Menu();
		}
		else if (sf::Keyboard::isKeyPressed(sf::Keyboard::P)) {
			map.TurnOffPellets();
		}
		else if (event.type == sf::Event::Closed) {
			window.close();
			return NULL;
		}
	}

	Updatedt();
	(this->*fToUpdate)();

	Render(window);

	return returnState;
}