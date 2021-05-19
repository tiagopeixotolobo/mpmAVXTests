#include "meshElementQ4.h"

void meshElementQ4::calculateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& v) {

    int flag = 1; //0 -> execução normal ~~~~~~ 1 -> execução avx 

    int nNodesInElem = nodeList.size();
    double	L = nodes[1]->pos(0) - nodes[0]->pos(0);

    if (!flag) { //normal
        for (unsigned int i = 0; i < nNodesInElem; ++i) {
            v(i) = oneDimensionalFormFunction(i, 0, x(0), lp, L) * oneDimensionalFormFunction(i, 1, x(1), lp, L);
        }
    }
    
    else { //avx
        __m256d vecX = _mm256_setr_pd(oneDimensionalFormFunction(0, 0, x(0), lp, L), oneDimensionalFormFunction(1, 0, x(0), lp, L), oneDimensionalFormFunction(2, 0, x(0), lp, L),
            oneDimensionalFormFunction(3, 0, x(0), lp, L));

        __m256d vecY = _mm256_setr_pd(oneDimensionalFormFunction(0, 1, x(1), lp, L), oneDimensionalFormFunction(1, 1, x(1), lp, L), oneDimensionalFormFunction(2, 1, x(1), lp, L),
            oneDimensionalFormFunction(3, 1, x(1), lp, L));

        __m256d res = _mm256_mul_pd(vecX, vecY);

        _mm256_store_pd(v.data(), res);
    }
}

void meshElementQ4::calculateDerivateFormFunction(const Eigen::Vector3d& x, double lp, Eigen::VectorXd& dv) {

    int flag = 1;

	int nNodesInElem = nodeList.size();
	double	L = nodes[1]->pos(0) - nodes[0]->pos(0);

    if (!flag) {
        for (unsigned int i = 0; i < nNodesInElem; ++i) {
            dv(i) = oneDimensionalFormFunctionDerivative(i, 0, x(0), lp, L) * oneDimensionalFormFunction(i, 1, x(1), lp, L);
            dv(i + nNodesInElem) = oneDimensionalFormFunction(i, 0, x(0), lp, L) * oneDimensionalFormFunctionDerivative(i, 1, x(1), lp, L);
        }
    }

    else {
        __m256d vecX = _mm256_setr_pd(oneDimensionalFormFunction(0, 0, x(0), lp, L), oneDimensionalFormFunction(1, 0, x(0), lp, L), oneDimensionalFormFunction(2, 0, x(0), lp, L),
            oneDimensionalFormFunction(3, 0, x(0), lp, L));

        __m256d vecY = _mm256_setr_pd(oneDimensionalFormFunction(0, 1, x(1), lp, L), oneDimensionalFormFunction(1, 1, x(1), lp, L), oneDimensionalFormFunction(2, 1, x(1), lp, L),
            oneDimensionalFormFunction(3, 1, x(1), lp, L));

        __m256d vecDX = _mm256_setr_pd(oneDimensionalFormFunctionDerivative(0, 0, x(0), lp, L), oneDimensionalFormFunctionDerivative(1, 0, x(0), lp, L), oneDimensionalFormFunctionDerivative(2, 0, x(0), lp, L),
            oneDimensionalFormFunctionDerivative(3, 0, x(0), lp, L));

        __m256d vecDY = _mm256_setr_pd(oneDimensionalFormFunctionDerivative(0, 1, x(1), lp, L), oneDimensionalFormFunctionDerivative(1, 1, x(1), lp, L), oneDimensionalFormFunctionDerivative(2, 1, x(1), lp, L),
            oneDimensionalFormFunctionDerivative(3, 1, x(1), lp, L));

        __m256d resXDY = _mm256_mul_pd(vecX, vecDY);
        __m256d resYDX = _mm256_mul_pd(vecDX, vecY);

        _mm256_store_pd(dv.data(), resXDY);
        _mm256_store_pd(dv.data() + nNodesInElem, resYDX);
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