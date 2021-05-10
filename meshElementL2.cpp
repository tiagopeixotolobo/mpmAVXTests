#include "meshElementL2.h"


void meshElementL2::calculateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& v) {

    double eta, Lx, Ly, L, ux, uy, u;
	Eigen::Vector3d projection;

	this->computePointInElementProjection(x, projection);

	Lx = this->nodes[1]->pos[0]-this->nodes[0]->pos[0];
	Ly = this->nodes[1]->pos[1]-this->nodes[0]->pos[1];

	ux = projection(0)-this->nodes[0]->pos[0];
	uy = projection(1)-this->nodes[0]->pos[1];

    L = std::sqrt(Lx * Lx + Ly * Ly);
    u = std::sqrt(ux * ux + uy * uy);

    eta = 2 * u/L - 1;

    v(0) = .5*(1 - eta);
    v(1) = .5*(1 + eta);

}

void meshElementL2::calculateDerivateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& dv) {

    double Lx, Ly, L;

    Lx = this->nodes[1]->pos[0]-this->nodes[0]->pos[0];
    Ly = this->nodes[1]->pos[1]-this->nodes[0]->pos[1];

    L = std::sqrt(Lx * Lx + Ly * Ly);

    if (this->nodes[0]->pos[0] < this->nodes[1]->pos[0]) {
        dv(0) = -1/L;
        dv(1) = 1/L;
        dv(2) = 0;
        dv(3) = 0;
    } else {
        dv(0) = 0;
        dv(1) = 0;
        dv(2) = -1/L;
        dv(3) = 1/L;
    }

}

void meshElementL2::computeGlobalCoordinates(double ksi, double eta, double zeta, Eigen::Vector3d& global) {

    double a, b, x[2];

    if (this->nodes[0]->pos[0] != this->nodes[1]->pos[0]) {

        x[0] = this->nodes[0]->pos[0];
        x[1] = this->nodes[1]->pos[0];

        a = 2 / (x[1] - x[0]);
        b = (x[0] + x[1]) / (x[0] - x[1]);

        global(0) = (ksi - b) / a;
        global(1) = this->nodes[0]->pos[1];
        global(2) = 0;

    } else {

        x[0] = this->nodes[0]->pos[1];
        x[1] = this->nodes[1]->pos[1];

        a = 2 / (x[1] - x[0]);
        b = (x[0] + x[1]) / (x[0] - x[1]);

        global(0) = this->nodes[0]->pos[0];
        global(1) = (ksi - b) / a;
        global(2) = 0;

    }

}

void meshElementL2::computePointInElementProjection(const Eigen::Vector3d &partPosition, Eigen::Vector3d &projection) {

    Eigen::Vector3d r1, r2, P;

    // Line points:
    r1 = this->nodes[0]->pos;
    r2 = this->nodes[1]->pos;

    // Outside point:
    P = partPosition;

    // Line coefficients:
    if (r2(0) != r1(0)) {

        double m = (r2(1) - r1(1)) / (r2(0) - r1(0));
        double n = r2(1) - m * r2(0);
        projection(0) = (m*(P(1)-n)+P(0))/(1+m*m);
        projection(1) = (m*m*P(1)+m*P(0)+n)/(1+m*m);

    } else {

        projection(0) = r1(0);
        projection(1) = P(1);

    }

    projection(2) = P(2);

}