#pragma once

#include "MPMparticle.h"
#include "mpm_meshRegularQ4.h"
#include <vector>
#include <iostream>

class dummyData {

	public:

		dummyData() {};
		~dummyData() {};

		//mesh data
		double x0mesh;
		double xfmesh;
		double y0mesh;
		double yfmesh;
		double xMeshAmplitude;
		double yMeshAmplitude;
		double backgroundMeshElemsize;
		int nElements;
		int meshType;
		std::vector<std::string> bodyLabels;

		//particles data
		double dx;
		double dy;
		double x0;
		double y0;
		double xf;
		double yf;
		double side;

		Eigen::Vector3d gravity;
		double density;
};

class engine
{
	public:

		engine();
		~engine();

		mpm::mpmMeshRegularQ4 *backgroundMesh;
		std::vector<mpmParticle*> particles;

		dummyData avxData;

		void startAVXData(double elemSide0, double dx0, double dy0);
		void setDummyParameters(double x0mesh, double xfmesh, double y0mesh, double yfmesh, double elemsize, double side, double dx, double dy);
		void startDummyMesh();
		void createParticles();
		void createElements();
		void createNodes();
		void mapParticles2Elements();
		void mapParticles2Nodes();
		void loadNodeList();
};

