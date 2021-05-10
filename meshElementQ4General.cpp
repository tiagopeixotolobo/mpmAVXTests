#include <iostream>
#include "meshElementQ4General.h"


void meshElementQ4General::computeCoefficients(double* coeff) {

    *(coeff)   = .25 * ( nodeList[0]->pos[0] + nodeList[1]->pos[0] + nodeList[2]->pos[0] + nodeList[3]->pos[0]);
    *(coeff+1) = .25 * (-nodeList[0]->pos[0] + nodeList[1]->pos[0] + nodeList[2]->pos[0] - nodeList[3]->pos[0]);
    *(coeff+2) = .25 * (-nodeList[0]->pos[0] - nodeList[1]->pos[0] + nodeList[2]->pos[0] + nodeList[3]->pos[0]);
    *(coeff+3) = .25 * ( nodeList[0]->pos[0] - nodeList[1]->pos[0] + nodeList[2]->pos[0] - nodeList[3]->pos[0]);
    *(coeff+4) = .25 * ( nodeList[0]->pos[1] + nodeList[1]->pos[1] + nodeList[2]->pos[1] + nodeList[3]->pos[1]);
    *(coeff+5) = .25 * (-nodeList[0]->pos[1] + nodeList[1]->pos[1] + nodeList[2]->pos[1] - nodeList[3]->pos[1]);
    *(coeff+6) = .25 * (-nodeList[0]->pos[1] - nodeList[1]->pos[1] + nodeList[2]->pos[1] + nodeList[3]->pos[1]);
    *(coeff+7) = .25 * ( nodeList[0]->pos[1] - nodeList[1]->pos[1] + nodeList[2]->pos[1] - nodeList[3]->pos[1]);

}

void meshElementQ4General::computeJacobian(const double* coeff, double* J) {

    *(J)   = coeff[1] + coeff[3] * eta;
    *(J+1) = coeff[2] + coeff[3] * ksi;
    *(J+2) = coeff[5] + coeff[7] * eta;
    *(J+3) = coeff[6] + coeff[7] * ksi;

}

void meshElementQ4General::computeJacobianInverse(const double* J, double* inv) {

    double denominator = J[0] * J[3] - J[1] * J[2];

    if (denominator != 0) {

        *(inv)   =  J[3] / denominator;
        *(inv+1) = -J[1] / denominator;
        *(inv+2) = -J[2] / denominator;
        *(inv+3) =  J[0] / denominator;

    } else {
        std::cerr << "The shape function's Jacobian matrix is not invertible." << std::endl;
    }
}

void meshElementQ4General::computeLocalCoordinates(const double* coeff, const Eigen::Vector3d& x) {

    ksi = 0;
    eta = 0;
    double J[4], f1, f2, deltaETA, deltaKSI;

    f1 = coeff[0] + coeff[1] * ksi + coeff[2] * eta + coeff[3] * eta * ksi - x(0);
    f2 = coeff[4] + coeff[5] * ksi + coeff[6] * eta + coeff[7] * eta * ksi - x(1);

    while (sqrt(f1*f1+f2*f2) > 1e-5) {

        computeJacobian(&coeff[0], &J[0]);

        deltaETA = (-f2 + f1 * J[2]/J[0])/(J[3] - J[2] * J[1]/J[0]);
        deltaKSI = (-f1 - J[1] * deltaETA)/J[0];

        ksi = ksi + deltaKSI;
        eta = eta + deltaETA;

        f1 = coeff[0] + coeff[1] * ksi + coeff[2] * eta + coeff[3] * eta * ksi - x(0);
        f2 = coeff[4] + coeff[5] * ksi + coeff[6] * eta + coeff[7] * eta * ksi - x(1);

    }

}

void meshElementQ4General::calculateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& v) {

    double coeff[8];

    computeCoefficients(&coeff[0]);
    computeLocalCoordinates(&coeff[0], x);

    v(0) = .25 * (1-ksi) * (1-eta);
    v(1) = .25 * (1+ksi) * (1-eta);
    v(2) = .25 * (1+ksi) * (1+eta);
    v(3) = .25 * (1-ksi) * (1+eta);

}

void meshElementQ4General::calculateDerivateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& dv) {

    double coeff[8], J[4], inv[4];

    computeCoefficients(&coeff[0]);
    computeLocalCoordinates(&coeff[0], x);
    computeJacobian(&coeff[0], &J[0]);
    computeJacobianInverse(&J[0], &inv[0]);

    dv(0) = -.25 * (inv[0] * (1-eta) + inv[1] * (1-ksi));
    dv(1) =  .25 * (inv[0] * (1-eta) - inv[1] * (1+ksi));
    dv(2) =  .25 * (inv[0] * (1+eta) + inv[1] * (1+ksi));
    dv(3) = -.25 * (inv[0] * (1+eta) - inv[1] * (1-ksi));

    dv(4) = -.25 * (inv[2] * (1-eta) + inv[3] * (1-ksi));
    dv(5) =  .25 * (inv[2] * (1-eta) - inv[3] * (1+ksi));
    dv(6) =  .25 * (inv[2] * (1+eta) + inv[3] * (1+ksi));
    dv(7) = -.25 * (inv[2] * (1+eta) - inv[3] * (1-ksi));

}