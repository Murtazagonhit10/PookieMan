#include "Menu.h"
#include "Playing.h"

Menu::Menu() : menuElemPlayClassic(0, &menuElemPlayNearVersion, &menuElemQuit), menuElemPlayNearVersion(1, &menuElemQuit, &menuElemPlayClassic),menuElemQuit(2, &menuElemPlayClassic, &menuElemPlayNearVersion){
	cout << "Menu consted" << endl;

	menuElemPlayClassic.text.setFont(font);
	menuElemPlayClassic.text.setString("      1. Play PookieMan");
	menuElemPlayClassic.text.setFillColor(Color::Yellow);
	menuElemPlayClassic.text.setCharacterSize(24);
	menuElemPlayClassic.text.setPosition(100,200);
	menuElemPlayClassic.text.setLetterSpacing(2);

	menuElemQuit.text.setFont(font);
	menuElemQuit.text.setString("2. Quit");
	menuElemQuit.text.setFillColor(Color::White);
	menuElemQuit.text.setCharacterSize(24);
	menuElemQuit.text.setPosition(280, 340);
	menuElemQuit.text.setLetterSpacing(2);

	DescriptionText.setFont(font);
	DescriptionText.setCharacterSize(17);
	DescriptionText.setPosition(2, 500);
	DescriptionText.setFillColor(Color::White);
	DescriptionText.setString("                                Short description:\n\n      Pac-Man 1980 PC videogame fan recreation by Near.\n                        Play the original Pac-Man game\n            with the original characters 1st menu point.\n                Or you could get to know two new ghosts\n                         Luxy and Darky 2nd menu point.\n                  You control pacman with the cursors.\n                                         Have fun.");

	temp = &menuElemPlayClassic;
	
}

Menu::~Menu()
{
	cout << "Menu deleted"<<endl;
}

GameState* Menu::Update(RenderWindow& window)
{
	while (window.pollEvent(event))
	{

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {

			temp->text.setFillColor(Color::White);
			temp = temp->next;
			temp->text.setFillColor(Color::Yellow);

		}

		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {


			temp->text.setFillColor(Color::White);
			temp = temp->previous;
			temp->text.setFillColor(Color::Yellow);


		}
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::Enter)) {

			switch (temp->num) {
			case 0:
				return new Playing(true);
				break;
			case 1:
				return new Playing(false);
				break;
			case 2:
				window.close();
				return NULL;
				break;

			}
		}
		if (event.type == sf::Event::Closed) {
			window.close();
			return NULL;
		}
	}
	
	if(window.isOpen())
		Render(window);

	return NULL;
}

void Menu::Render(RenderWindow& window) {

	window.clear();
	window.draw(menuElemPlayClassic.text);
	window.draw(menuElemPlayNearVersion.text);
	window.draw(menuElemQuit.text);
	window.draw(DescriptionText);
	window.display();
}
