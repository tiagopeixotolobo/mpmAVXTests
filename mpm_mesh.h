#ifndef MPM_MESH_H
#define MPM_MESH_H

#include <vector>
#include "mpm_node.h"
#include "mpm_meshelement.h"
#include "meshElementQ4GIMP.h"
#include "meshElementQ4Traditional.h"
#include "meshElementQ4General.h"
#include "meshElementT3.h"
#include "meshElementL2.h"
#include "mpm_body.h"
#include "MPMParticle.h"


class mpmMesh
{
public:
	mpmMesh(std::vector<std::string>);
	~mpmMesh();

	int nElem = 0;
    int nx, ny;
    double xLim[2], yLim[2];
    double minElemSize[2] = {0, 0};

    void addElemL2(int, int[2]);
    void addElemT3(int, int[3]);
    void addElemQ4Traditional(int, int[4]);
    void addElemQ4GIMP(int, int[4]);
	void addElemQ4General(int, int[4]);

    void addLowerDimensionElement(meshElement*, int, int[2]); // TO DO: create a version of this method for each child of meshElement.

    void addNode(int, double, double, double, bool, bool, bool);
    void resetGrid();
    void resetMassVelocityMomentum();

    virtual void initializeMeshVariables() = 0;
	virtual int mapParticle2Element(mpmParticle*) = 0;

    std::string getBodyLabelFromID(int);
    int getBodyIDFromLabel(const std::string&);

	std::vector<mpm::mpmBody*> body;
};



#endif

