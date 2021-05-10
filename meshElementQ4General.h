#pragma once
#include "meshElementQ4.h"
#include "Eigen/Core"

class meshElementQ4General :
	public meshElementQ4
{
public:
    using meshElementQ4::meshElementQ4;
    void calculateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) override;
    void calculateDerivateFormFunction(const Eigen::Vector3d&, double, Eigen::VectorXd&) override;
    double oneDimensionalFormFunction(int a, int b, double c, double d, double e) override { return 0; };
    double oneDimensionalFormFunctionDerivative(int a, int b, double c, double d, double e) override { return 0; };

private:
    double ksi, eta;
    void computeCoefficients(double*);
    void computeJacobian(const double*, double*);
    void computeJacobianInverse(const double*, double*);
    void computeLocalCoordinates(const double*, const Eigen::Vector3d&);
};

