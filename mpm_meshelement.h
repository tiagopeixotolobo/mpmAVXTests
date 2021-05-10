#pragma once

#include <vector>
#include "Eigen/Core"
#include "mpm_globals.h"
#include "mpm_node.h"


class meshElement {
public:
	meshElement(int);
	~meshElement();

	std::vector<node*> nodes;
	std::vector<node*> nodeList;
    std::vector<int> boundaryParticlesIDs;

    Eigen::Vector3d traction;

	double xLimits[2];
    double yLimits[2];
	double volumeCorrected;
	double volume;

	bool active;
	bool isInternal;

	int idElem;
	int type;

    virtual double computeArea();
    virtual double computeVolume();
	virtual void computeLimits();
	virtual void computeGlobalCoordinates(double, double, double, Eigen::Vector3d&) = 0;
	virtual void calculateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) = 0;
	virtual void calculateDerivateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) = 0;
};