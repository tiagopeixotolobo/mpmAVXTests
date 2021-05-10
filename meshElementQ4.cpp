#include "meshElementQ4.h"


void meshElementQ4::calculateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& v) {

	int nNodesInElem = nodeList.size();
	double	L = nodes[1]->pos(0) - nodes[0]->pos(0);

	for (unsigned int i = 0; i < nNodesInElem; ++i) {
		v(i) = oneDimensionalFormFunction(i, 0, x(0), lp, L)*oneDimensionalFormFunction(i, 1, x(1), lp, L);
	}

}

void meshElementQ4::calculateDerivateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& dv) {

	int nNodesInElem = nodeList.size();
	double	L = nodes[1]->pos(0) - nodes[0]->pos(0);

	for (unsigned int i = 0; i < nNodesInElem; ++i) {
		dv(i) = oneDimensionalFormFunctionDerivative(i, 0, x(0), lp, L)*oneDimensionalFormFunction(i, 1, x(1), lp, L);
		dv(i + nNodesInElem) = oneDimensionalFormFunction(i, 0, x(0), lp, L)*oneDimensionalFormFunctionDerivative(i, 1, x(1), lp, L);
	}
}

void meshElementQ4::computeGlobalCoordinates(double ksi, double eta, double zeta, Eigen::Vector3d& global) {

    double a[2], b[2], x[2], y[2];

    x[0] = this->nodeList[0]->pos[0];
    x[1] = this->nodeList[1]->pos[0];

    y[0] = this->nodeList[0]->pos[1];
    y[1] = this->nodeList[3]->pos[1];

    a[0] = 2 / (x[1] - x[0]);
    a[1] = 2 / (y[1] - y[0]);

    b[0] = (x[0] + x[1]) / (x[0] - x[1]);
    b[1] = (y[0] + y[1]) / (y[0] - y[1]);

    global(0) = (ksi - b[0]) / a[0];
    global(1) = (eta - b[1]) / a[1];
    global(2) = 0;

}