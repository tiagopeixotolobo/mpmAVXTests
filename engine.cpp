#include "engine.h"

void engine::startAVXData()
{

	setDummyParameters();
	startDummyMesh(10,10,0.1);



}

void engine::startDummyMesh(double L, double H, double elemsize)
{
	this->avxData.meshXDimension = L;
	this->avxData.meshYDimension = H;
	this->avxData.backgroundMeshElemsize = elemsize;

}


void setDummyParameters() {



}