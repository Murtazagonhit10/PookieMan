#include <stack>
#include "GameState.h"
#include "Menu.h"
#include "Playing.h"

using std::stack;

class Game {

public:

	Game();
	~Game();
	void run();
private:

	float dt;
	RenderWindow window;
	Event event;

	GameState* gameState;
	GameState* newStateToSet;
};
