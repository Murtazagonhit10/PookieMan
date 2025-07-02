#include "Ghost.h"

#ifndef H_CLYDE
#define H_CLYDE
#define CLYDESTARTX 15.70
#define CLYDESTARTY 14

class Clyde :public Ghost
{

public:

	Clyde();
	~Clyde();
	void Update(const float& dt);
	void setChaseTargetNode() override;
	void setScatterTargetNode() override;
	void SetStartState() override;
	void SetStartParams() override;
	Vector2i getTempCoordsOnLevel();

	static Vector2i sTempCoordsOnLevel;
	void moveUpAndDown() override;
	bool moveToFourteenDotThirtyFive() override;
	unsigned short int GetActivationDotLimit() override;
private:
	/* Constants start */

	const Vector2i scatterTargetNode = { 0,34 };

	/* Constants end */
};

#endif
