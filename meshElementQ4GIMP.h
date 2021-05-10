#pragma once
#include "meshElementQ4.h"
class meshElementQ4GIMP :
	public meshElementQ4
{
public:
	using meshElementQ4::meshElementQ4;
	double oneDimensionalFormFunction(int, int, double, double, double);
	double oneDimensionalFormFunctionDerivative(int, int, double, double, double);

};

