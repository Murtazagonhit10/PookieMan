#pragma once
#include "LevelVariables.h"

constexpr unsigned short int percentage(unsigned short int n, unsigned short int p) //p % of the n number
{
	return n * ((float)p/100);
}

/* This table contains the following informations:

The row number equals to the number of the level. The 12 columns contains the following inormations:

----------0------------------1-------------2----------------3--------------4--------------5-----------------6---------------7---------------8-----------------9---------------------10---------------------11------------------12------------------13------------------14----------
BonusFruitPictureNumber|Bonus Points|Pac-Man Speed|Pac-Man Dots Speed|Ghost Speed|GhostTunnel Speed|Elroy 1 Dots Left|Elroy 1 Speed|Elroy 2 Dots Left|Elroy 2 Speed| Fright. Pac-Man Speed| Fright Pac-Man Dots Speed|Fright Ghost Speed|Fright Time in Sec.|Number of flashes

percentage does a % calculation
every speed is calculated from the basic speed value of the characters
fruits: are numbered from 0 to 6 which is the order of the images on the texture sprite sheet
value 0 means, no value change in the speed limit or no limit like the frighten mode in the last rows has no time limit because there is no frighten mode anymore on those levels.
*/
//20 rows
unsigned short int levelValues[][15] =
{ {0,100,percentage(PACMANBASICSPEED,80),percentage(PACMANBASICSPEED,71),percentage(GHOSTBASICSPEED,75),percentage(GHOSTBASICSPEED,40),20,percentage(GHOSTBASICSPEED,80),10,percentage(GHOSTBASICSPEED,90),percentage(PACMANBASICSPEED,90),percentage(PACMANBASICSPEED,79),percentage(GHOSTBASICSPEED,50),6,5}, //x
{1,300,percentage(PACMANBASICSPEED,90),percentage(PACMANBASICSPEED,79),percentage(GHOSTBASICSPEED,85),percentage(GHOSTBASICSPEED,45),30,percentage(GHOSTBASICSPEED,90),15,percentage(GHOSTBASICSPEED,95),percentage(PACMANBASICSPEED,95),percentage(PACMANBASICSPEED,83),percentage(GHOSTBASICSPEED,55),5,5},//x
{2,500,percentage(PACMANBASICSPEED,90),percentage(PACMANBASICSPEED,79),percentage(GHOSTBASICSPEED,85),percentage(GHOSTBASICSPEED,45),40,percentage(GHOSTBASICSPEED,90),20,percentage(GHOSTBASICSPEED,95),percentage(PACMANBASICSPEED,95),percentage(PACMANBASICSPEED,83),percentage(GHOSTBASICSPEED,55),4,5},//x
{2,500,percentage(PACMANBASICSPEED,90),percentage(PACMANBASICSPEED,79),percentage(GHOSTBASICSPEED,85),percentage(GHOSTBASICSPEED,45),40,percentage(GHOSTBASICSPEED,90),20,percentage(GHOSTBASICSPEED,95),percentage(PACMANBASICSPEED,95),percentage(PACMANBASICSPEED,83),percentage(GHOSTBASICSPEED,55),3,5},//x
{3,700,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),40,percentage(GHOSTBASICSPEED,100),20,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),2,5},//x
{3,700,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),50,percentage(GHOSTBASICSPEED,100),25,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),5,5},//x
{4,1000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),50,percentage(GHOSTBASICSPEED,100),25,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),2,5},//x
{4,1000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),50,percentage(GHOSTBASICSPEED,100),25,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),2,5},//x
{5,2000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),60,percentage(GHOSTBASICSPEED,100),30,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),1,3},//x
{5,2000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),60,percentage(GHOSTBASICSPEED,100),30,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),5,5},//x
{6,3000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),60,percentage(GHOSTBASICSPEED,100),30,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),2,5},//x
{6,3000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),80,percentage(GHOSTBASICSPEED,100),40,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),1,3},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),80,percentage(GHOSTBASICSPEED,100),40,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),1,3},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),80,percentage(GHOSTBASICSPEED,100),40,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),3,5},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),100,percentage(GHOSTBASICSPEED,100),50,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),1,3},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),100,percentage(GHOSTBASICSPEED,100),50,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),1,3},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),100,percentage(GHOSTBASICSPEED,100),50,percentage(GHOSTBASICSPEED,105),0,0,0,0,0},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),100,percentage(GHOSTBASICSPEED,100),50,percentage(GHOSTBASICSPEED,105),percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,60),1,3},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),120,percentage(GHOSTBASICSPEED,100),60,percentage(GHOSTBASICSPEED,105),0,0,0,0,0},//x
{7,5000,percentage(PACMANBASICSPEED,100),percentage(PACMANBASICSPEED,87),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),120,percentage(GHOSTBASICSPEED,100),60,percentage(GHOSTBASICSPEED,105),0,0,0,0,0},//x
{7,5000,percentage(PACMANBASICSPEED,90),percentage(PACMANBASICSPEED,79),percentage(GHOSTBASICSPEED,95),percentage(GHOSTBASICSPEED,50),120,percentage(GHOSTBASICSPEED,100),60,percentage(GHOSTBASICSPEED,105),0,0,0,0,0}//x
};

float scatterTimings[4][5]{
	{7,7,7,7,5},
	{7,7,7,7,5},
	{5,5,5,5,5},
	{0,0,0,0,0}
};

/* 0 means infinite */

unsigned short int chaseTimings[4][5]{
	{20,20,20,20,20},
	{20,20,20,20,20},
	{20,1033,1033,1033,1037},
	{0,0,0,0,0}
};

unsigned short int fruitAppear[22] = { 170 ,75,170 ,75, 170 ,75, 170 ,75, 165, 75,165, 75, 165, 75, 165, 75, 165, 75, 165, 75, 165, 75 };

unsigned short int LEVELNUMBER = 2; //0. is the first level

/* Level variables */
bool Game_Over = false;
bool Game_Win = false;
bool paused = true;
bool elroy1 = OFF;
bool elroy2 = OFF;
bool SpecialCounter = OFF;

Font font;

unsigned short int GlobalCounter = 0;
