#pragma once
#include "mpm_meshelement.h"
#include "mpm_globals.h"

// Kelvin-Foigt element
class meshElementL2 : public meshElement
{
public:
	meshElementL2(int id):meshElement(id) { this->idElem = id; this->type = ELEMENT_L2; };
    ~meshElementL2() {};
    void computePointInElementProjection(const Eigen::Vector3d&, Eigen::Vector3d&);
    void computeGlobalCoordinates(double, double, double, Eigen::Vector3d&) ;
	void calculateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) ;
	void calculateDerivateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) ;
};