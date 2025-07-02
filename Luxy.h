#pragma once
#include "Ghost.h"

#define LUXYSTARTX 13
#define LUXYSTARTY 11

class Luxy :
	public Ghost
{
public:


	Luxy();
	~Luxy();
	void Update(const float& dt);
	void setChaseTargetNode() override;
	void setScatterTargetNode() override;
	void SetStartState() override;
	void SetStartParams() override;
	Vector2i getTempCoordsOnLevel() const;
	static Vector2i sTempCoordsOnLevel;

	unsigned short int GetActivationDotLimit() override;
	void calculateNewDirection() override;
	void calculateDirectionTowardsDarky();
	

private:
	/* Constants start */
	Vector2i DarkyTempCoords;
	Vector2i PacmanTempDirection;
	bool chaseDarky;
	/* Constants end */
};

