#include "meshElementQ4GIMP.h"


double meshElementQ4GIMP::oneDimensionalFormFunction(int nodeId, int dir, double xp, double lp, double L) {

	double S = 0;
	double x = xp - nodeList[nodeId]->pos(dir);


	if ((-L - lp < x) && (x <= -L + lp)) {
		S = (L + lp + x) * (L + lp + x) / (4 * L*lp);
	}
	else {
		if ((-L + lp < x) && (x <= -lp)) {
			S = 1 + x / L;
		}
		else {
			if ((-lp < x) && (x <= lp)) {
				S = 1 - ( x * x + lp * lp) / (2 * L*lp);
			}
			else {
				if ((lp < x) && (x <= L - lp)) {
					S = 1 - x / L;
				}
				else {
					if ((L - lp < x) && (x <= L + lp)) {
						S = (L + lp - x)*(L + lp - x) / (4 * L*lp);
					}
				}
			}
		}
	}
	return S;
}

double meshElementQ4GIMP::oneDimensionalFormFunctionDerivative(int nodeId, int dir, double xp, double lp, double L) {

	double S = 0;
	double x = xp - nodeList[nodeId]->pos(dir);

	if ((-L - lp < x) && (x <= -L + lp)) {
		S = (L + lp + x) / (2 * L*lp);
	}
	else {
		if ((-L + lp < x) && (x <= -lp)) {
			S = 1 / L;
		}
		else {
			if ((-lp < x) && (x <= lp)) {
				S = -x / (L*lp);
			}
			else {
				if ((lp < x) && (x <= L - lp)) {
					S = -1 / L;
				}
				else {
					if ((L - lp < x) && (x <= L + lp)) {
						S = -(L + lp - x) / (2 * L*lp);
					}
				}
			}
		}
	}
	return S;
}