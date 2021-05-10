#include "meshElementT3.h"


void meshElementT3::calculateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& v) {

    double alpha, beta, delta, k[4];

    delta =   (nodeList[1]->pos(0) - nodeList[0]->pos(0)) * (nodeList[2]->pos(1) - nodeList[0]->pos(1))
            - (nodeList[2]->pos(0) - nodeList[0]->pos(0)) * (nodeList[1]->pos(1) - nodeList[0]->pos(1));

    k[0] =  1/delta * (nodeList[2]->pos(1) - nodeList[0]->pos(1));
    k[1] =  1/delta * (nodeList[2]->pos(0) - nodeList[0]->pos(0));
    k[2] = -1/delta * (nodeList[1]->pos(1) - nodeList[0]->pos(1));
    k[3] = -1/delta * (nodeList[1]->pos(0) - nodeList[0]->pos(0));

    alpha =  k[0] * (x(0) - nodeList[0]->pos(0))
            -k[1] * (x(1) - nodeList[0]->pos(1));

    beta  =  k[2] * (x(0) - nodeList[0]->pos(0))
            -k[3] * (x(1) - nodeList[0]->pos(1));

    v(0) = 1 - alpha - beta;
    v(1) = alpha;
    v(2) = beta;

}

void meshElementT3::calculateDerivateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& dv) {

    double delta, k[4];

    delta =   (nodeList[1]->pos(0) - nodeList[0]->pos(0)) * (nodeList[2]->pos(1) - nodeList[0]->pos(1))
            - (nodeList[2]->pos(0) - nodeList[0]->pos(0)) * (nodeList[1]->pos(1) - nodeList[0]->pos(1));

    k[0] =  1/delta * (nodeList[2]->pos(1) - nodeList[0]->pos(1));
    k[1] =  1/delta * (nodeList[2]->pos(0) - nodeList[0]->pos(0));
    k[2] = -1/delta * (nodeList[1]->pos(1) - nodeList[0]->pos(1));
    k[3] = -1/delta * (nodeList[1]->pos(0) - nodeList[0]->pos(0));

    // Derivatives with respect to x:
    dv(0) = -k[0] - k[2];
    dv(1) =  k[0];
    dv(2) =  k[2];

    // Derivatives with respect to y:
    dv(3) =  k[1] + k[3];
    dv(4) = -k[1];
    dv(5) = -k[3];

}

void meshElementT3::computeGlobalCoordinates(double ksi, double eta, double zeta, Eigen::Vector3d& global) {

    double delta, k[4];

    delta =   (nodeList[1]->pos(0) - nodeList[0]->pos(0)) * (nodeList[2]->pos(1) - nodeList[0]->pos(1))
            - (nodeList[2]->pos(0) - nodeList[0]->pos(0)) * (nodeList[1]->pos(1) - nodeList[0]->pos(1));

    k[0] =  1/delta * (nodeList[2]->pos(1) - nodeList[0]->pos(1));
    k[1] =  1/delta * (nodeList[2]->pos(0) - nodeList[0]->pos(0));
    k[2] = -1/delta * (nodeList[1]->pos(1) - nodeList[0]->pos(1));
    k[3] = -1/delta * (nodeList[1]->pos(0) - nodeList[0]->pos(0));

    global(0) =  (k[3] * eta - k[1] * ksi) / (k[3] * k[0] - k[2] * k[1]) + nodeList[0]->pos(0);
    global(1) = -(k[0] * eta - k[2] * ksi) / (k[3] * k[0] - k[2] * k[1]) + nodeList[0]->pos(1);
    global(2) = 0;

}

