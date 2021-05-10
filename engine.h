#pragma once

#include "MPMparticle.h"
#include "mpm_mesh.h"
#include <vector>

class dummyData {
	dummyData() {};
	~dummyData() {};

public:
	double meshXDimension;
	double meshYDimension;
	double backgroundMeshElemsize;
};

class engine
{
public:

	mpmMesh *backgroundMesh = nullptr;
	std::vector<mpmParticle*> particles;

	dummyData avxData;

	void startAVXData();
	void setDummyParameters();
	void startDummyMesh(double L, double H, double elemsize);
};

