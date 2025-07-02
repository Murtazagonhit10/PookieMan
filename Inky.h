/* Blinky wandering after PACMAN, if Blinky gets into a short range of Pacman,

he gets aggressive and starts chasing until he catches PACMAN or PACMAN gets out of the range. */

#include "Ghost.h"
#ifndef H_INKY
#define H_INKY

#define INKYSTARTX 11.40f //13.60
#define INKYSTARTY 14.0f //13.60

class Inky :public Ghost
{

public:

	Inky();
	~Inky();
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
	Vector2i blinkyTempCoordsOnLevel;
	unsigned short int DotCounter;

	

private:
	/* Constants start */
	//const float INKYSTARTX = 11.40f; //13.60
	//const float INKYSTARTY = 14.0f;

	const Vector2i scatterTargetNode = { 27,34 };

	Texture targettexture;
	Texture InkyTexture;
	/* Constants end */
};

#endif