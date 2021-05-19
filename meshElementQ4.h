#pragma once
#include "mpm_meshelement.h"
#include "mpm_globals.h"
#include <immintrin.h>

class meshElementQ4 : public meshElement
{
public:
	meshElementQ4(int id):meshElement(id) { this->idElem = id; this->type = ELEMENT_Q4; };
    ~meshElementQ4() {};
    void computeGlobalCoordinates(double, double, double, Eigen::Vector3d&) ;
	void calculateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) ;
	void calculateDerivateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) ;
	virtual double oneDimensionalFormFunction(int, int, double, double, double) = 0;
	virtual double oneDimensionalFormFunctionDerivative(int, int, double, double, double) = 0;
};