/* Blinky wandering after PACMAN, if Blinky gets into a short range of Pacman,

he gets aggressive and starts chasing until he catches PACMAN or PACMAN gets out of the range. */

#include "Ghost.h"
#ifndef H_PINKY
#define H_PINKY
#define PINKYTEXTUREPATH "Textures/pinky.png"


#define PINKYSTARTXCLASS 13.60f //13.60
#define PINKYSTARTYCLASS 14.0f //13.60
#define PINKYSTARTX 11.40f //13.60
#define PINKYSTARTY 14.0f //13.60

class Pinky :public Ghost
{

public:

	Pinky(const bool& classic);
	~Pinky();
	void Update(const float& dt);
	void SetStartState() override;
	void moveUpAndDown() override;
	bool moveToFourteenDotThirtyFive() override;
	void setChaseTargetNode() override;
	void setScatterTargetNode() override;
	void SetStartParams() override;
	unsigned short int GetActivationDotLimit() override;
	
public:
	Vector2i pacmanTempDirection;

private:
	/* Constants start */

	const Vector2i scatterTargetNode = { 0,0 };

	/* Constants end */

	Texture PinkyTexture;
	Texture targettexture;


	unsigned short int DotCounter;
};

#endif