#pragma once
#include "GameState.h"
class Menu : public GameState
{
public:
	Menu();
	~Menu();
	GameState* Update(RenderWindow&);
	void Render(RenderWindow& );
public:
	typedef struct MenuElem {
	
	public:
		MenuElem(unsigned short int pnum, MenuElem* pnext, MenuElem* pprevious) :num(pnum), next(pnext), previous(pprevious) {}
	public:
		MenuElem* previous;
		MenuElem* next;
		unsigned short int num;
		Text text;
	}MenuElem;

private:

	MenuElem menuElemPlayClassic;
	MenuElem menuElemPlayNearVersion;
	MenuElem menuElemQuit;
	MenuElem* temp;

	Text DescriptionText;

	Event event;
};

