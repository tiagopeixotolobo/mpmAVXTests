#pragma once
#include "mpm_meshelement.h"
#include "mpm_globals.h"

class meshElementT3 : public meshElement
{
public:
    meshElementT3(int id):meshElement(id) { this->idElem = id; this->type = ELEMENT_T3; };
    void computeGlobalCoordinates(double, double, double, Eigen::Vector3d&) ;
	void calculateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) ;
	void calculateDerivateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) ;
};