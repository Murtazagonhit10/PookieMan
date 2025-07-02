#pragma once
#include "Ghost.h"

#define DARKYSTARTX 13.60f //13.60
#define DARKYSTARTY 14.0f

class Darky :
	public Ghost
{
public:


	Darky();
	~Darky();
	void Update(const float& dt);
	void setChaseTargetNode() override;
	void setScatterTargetNode() override;
	void SetStartState() override;
	void SetStartParams() override;
	Vector2i getTempCoordsOnLevel() const;
	static Vector2i sTempCoordsOnLevel;
	unsigned short int GetActivationDotLimit() override;
	bool moveToFourteenDotThirtyFive() override;
	void moveUpAndDown() override;

	void calculateNewDirection() override;
	void calculateDirectionTowardsLuxy();
private:
	Vector2i getPelletInFrontOfPacman();
private:
	/* Constants start */
	Vector2i LuxyTempCoords;
	Vector2i PacmanTempDirection;
	
	bool chaseLuxy;

	/* Constants end */
};

