#include "Animation.h"
#include <iostream>

Animation::Animation(Pacman*)
{
	totalTime = 0.0f;
	//if (pacman) {
		AimageCount = { 2, 4 };
		
		currentImage.x = 0;
		currentImage.y = 0;
		uvRect.width = 14.0f;
		uvRect.height = 14.0f;
	//}
}

Animation::Animation(Ghost*)
{
	totalTime = 0;
	selectBox = {16,16}; //default 16x16 for ghosts
	uvRect.width = 14;
	uvRect.height = 14;
}

Animation::~Animation()
{
}

void Animation::Update(const float& deltaTime,const float& switchTime) { //which row = imageToSet.y

	totalTime += deltaTime;

	if (totalTime >= switchTime) {
		totalTime = 0;
		++imageToSet.x;
		if (imageToSet.x > lastImage) {
			imageToSet.x = firstImage;
		}
		
	}

	uvRect.left = (imageToSet.x * selectBox.x);

	uvRect.top = (imageToSet.y * selectBox.y);
}

void Animation::UpdateFrightenAnimation(const float& deltaTime, const float& switchTime, unsigned short int& animationCounter)
{
	totalTime += deltaTime;

	if (totalTime >= switchTime) {
		totalTime = 0;
		++imageToSet.x;
		if (imageToSet.x > lastImage) {
			imageToSet.x = firstImage;
			++animationCounter;
		}

	}

	uvRect.left = (imageToSet.x * selectBox.x);

	uvRect.top = (imageToSet.y * selectBox.y);
}

void Animation::Update(const int& row, const float& deltaTime, const float& switchTime)
{

	currentImage.y = row;
	totalTime += deltaTime;

	if (totalTime >= switchTime) {

		totalTime = 0;
		currentImage.x++;

		if (currentImage.x >= AimageCount.x) {
			currentImage.x = 0;
		}
	}

	uvRect.left = currentImage.x * uvRect.width;
	uvRect.top = currentImage.y * uvRect.height;

}

int Animation::UpdateCustomOfColumns(int row, int col, const float& deltaTime, float switchTime) //This function is to update with custom number of columns
{
	unsigned short int tempimgnum;
	currentImage.y = row;
	totalTime += deltaTime;

	if (totalTime >= switchTime) {

		totalTime = 0;
		currentImage.x++;

		if (currentImage.x >= col) {
			tempimgnum = currentImage.x;
			currentImage.x = 0;
			return tempimgnum;
		}
	}

	uvRect.left = currentImage.x * uvRect.width;
	uvRect.top = currentImage.y * uvRect.height;

	return 0;
}

void Animation::UpdateSingleImage()
{

	uvRect.left = imageToSet.x * selectBox.x;
	uvRect.top = imageToSet.y * selectBox.y;

}

void Animation::setScoreImage(const unsigned short int& scoreNum) 
{

	uvRect.height = 8;
	uvRect.top = 68;
	switch (scoreNum) {
	
	case 0:
		
		uvRect.left = 0;
		uvRect.width = 14;
		break;
	case 1:
		
		uvRect.left = 16;
		uvRect.width = 15;
		break;
	case 2:
		
		uvRect.left = 33;
		uvRect.width = 15;
		break;
	case 3:
		
		uvRect.left = 50;
		uvRect.width = 16;
		break;
	}
}


