#include "meshElementQ4Traditional.h"


double meshElementQ4Traditional::oneDimensionalFormFunction(int nodeId, int dir, double xp, double lp, double L) {
	
	double S = 0;
	double x = xp - nodeList[nodeId]->pos(dir);

	if ((x > -L) && (x <= 0)) {
		S = 1 + x / L;
	}
	else {
		if ((x > 0) && (x <= L)) {
			S = 1 - x / L;
		}
	}
	return S;

}

double meshElementQ4Traditional::oneDimensionalFormFunctionDerivative(int nodeId, int dir, double xp, double lp, double L) {

	double dS = 0;
	double x = xp - nodeList[nodeId]->pos(dir);

	if ((x > -L) && (x <= 0)) {
		dS = 1 / L;
	}
	else {
		if ((x > 0) && (x <= L)) {
			dS = -1 / L;
		}
	}

	return dS;

}
