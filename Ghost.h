#include "Animation.h" //Animation includes the Grahpics.hpp
//#include "ActorState.h"

#include <stack>
#include "ActorState.h"
#include "SFML\Audio.hpp"

using namespace std;

//enum GhostState; //Ghost is constructed earlier, forward declaration needed
class ActorState;
enum GhostState;

#ifndef H_GHOST
#define H_GHOST

#ifndef CELLSIZE
#define CELLSIZE 24
#endif

#define SCORETEXTUREROW 4
#define EYEBALLSTEXTUREROW 1

class Ghost
{
public:
	bool collideWithPacman();
	Ghost();
	virtual ~Ghost();
	virtual void Update(const float&) = 0;
	void Draw(RenderWindow&);
	void setTargetNode(const Vector2i& target);

	virtual void setChaseTargetNode() = 0;
	virtual void setScatterTargetNode() = 0;
	virtual void SetStartState() = 0;
	virtual void SetStartParams() = 0;
	virtual unsigned short int GetActivationDotLimit() = 0;
	unsigned short int GetGhostPriorityNumber() const{ return PriorityNumber; }

	void setSpeed(float speed_) { speed = speed_; }
	Vector2i getTargetNode() const { return targetNode; }

	/* New movement version */
	bool turningPointReached();
	bool tunnelPointReached(); //Tunnel point is marked with the character 's' on the Map.
	virtual void calculateNewDirection();
	void calculateNewDirectionEatenMode();
	/* ******************** */

	void turnAround();
	void setDirection();
	void setDirection(Vector2i);

	void setState(ActorState* pstateToSet) { //params: 1. The state you want to set
		delete state;
		state = pstateToSet;
	}

	void SetFrightenMode() {
		isFrightened ? frightenedAgain = true : isFrightened = true;
	}

	virtual void moveUpAndDown() { return; } //really amateur solution to move up and down in the ghost house, returns true if there was a direction change.
	virtual bool moveToFourteenDotThirtyFive() { return true; } //really amateur solution to get the ghost into the middle of the ghost house
	bool comeOutFromHouse();
	void chooseRandomDirection();

	void playEatenSound();

	unsigned short int getDirectionForAnimation();
	
	Vector2f ghostTempPosition() const;
	sf::Vector2i ghostTempCorrdinate() const {

		sf::Vector2i ghostTempCoord;
		ghostTempCoord.x = (int)(ghostBody.getPosition().x / CELLSIZE);
		ghostTempCoord.y = (int)(ghostBody.getPosition().y / CELLSIZE);
		return ghostTempCoord;
	}
	void moveOn(const float& dt);
	bool tunnelTeleport();
	void UpdateTexture();
	void setStartPosition();



public:
	Vector2i ghostHouseStartNode;

	bool isFrightened;
	bool frightenedAgain;

	
	bool ActivateGhost;
	bool visible;
	bool inTunnel;

	float ANIMATIONSWITCHTIME = 0.25f;

	unsigned short int rowForAnimation; //Which row on the Texture.
	unsigned short int ChaseStateCounter; //How many times have been in this state
	unsigned short int ScatterStateCounter;//How many times have been in this state
	unsigned short int counterLimit;//After how many dots to come out
	unsigned short int PriorityNumber;//GhostHouse

	int speed;

	Vector2i startDirection;

	Vector2f startPoints;

	GhostState currentState;
	Animation animation;
	static Texture ghostTexture;

	static Text scoreText;


protected:
	/* constants start */
	const float GHOSTBODYSIZE = 40.0f;
	const float TURNNZONELOWERBOUND = 0.40f;
	const float TURNNZONEUPPERBOUND = 0.60f;
	const Vector2u imageCount = {2,8};
	
	/* constants end */
	

	RectangleShape ghostBody;

	RectangleShape targetMark;

	ActorState* state;

	stack<Vector2i> path;

	Vector2i direction;  //temporary direction
	Vector2i targetNode;
	Vector2i directionNode;
	
	Vector2i pacManTempCoordsOnLevel; //needed for chasing


protected:

	int manhattanDistance(int x1, int y1, int x2, int y2)
	{
		// Calculating distance 
		return abs(x2 - x1) +
			abs(y2 - y1);
	}
	
	float eucledianDistance(int x1, int y1, int x2, int y2) {
		return sqrt(pow((x2-x1),2) + pow((y2 - y1), 2));
	}
	

private:
	Vector2i previousTurningpoint;

	static SoundBuffer eaten_soundbuffer;
	static Sound eaten_sound;
};



#endif
