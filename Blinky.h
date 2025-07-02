/* Blinky wandering after PACMAN, if Blinky gets into a short range of Pacman,

he gets aggressive and starts chasing until he catches PACMAN or PACMAN gets out of the range. */

#include "Ghost.h"

#ifndef H_BLINKY
#define H_BLINKY

#define BLINKYSTARTX 13
#define BLINKYSTARTY 11
#define BLINKYSTARTXCLASS 13.60f //13.60
#define BLINKYSTARTYCLASS 14.0f //13.60

class Blinky:public Ghost
{

public:

	Blinky();
	~Blinky();
	void Update(const float& dt);
	void setChaseTargetNode() override;
	void setScatterTargetNode() override;
	void SetStartState() override;
	void SetStartParams() override;
	Vector2i getTempCoordsOnLevel() const;
	static Vector2i sTempCoordsOnLevel;
	unsigned short int GetActivationDotLimit() override;
private:
	/* Constants start */

	const Vector2i scatterTargetNode = {23,4};
	
	/* Constants end */
};

#endif