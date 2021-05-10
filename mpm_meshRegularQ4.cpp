#include "meshes/mpm_meshRegularQ4.h"
#include <iostream>

using namespace mpm;

int mpmMeshRegularQ4::mapParticle2Element(mpmParticle *part) {

    double idPart[2];
    int idElem = -1;

    idPart[0] = (part->position[0] - xLim[0]) / elemSize[0];
    idPart[0] = ceil(idPart[0]);
    idPart[1] = (part->position[1] - yLim[0]) / elemSize[1];
    idPart[1] = ceil(idPart[1]);
    idElem = (int)((idPart[1] - 1) * (nx) + idPart[0] - 1);

    return idElem;

}

void mpmMeshRegularQ4::initializeMeshVariables() {

    nElem = body[0]->elements.size();

    xLim[0] = body[0]->nodes[0]->pos[0];
    yLim[0] = body[0]->nodes[0]->pos[1];

    xLim[1] = body[0]->nodes[body[0]->nodes.size()-1]->pos[0];
    yLim[1] = body[0]->nodes[body[0]->nodes.size()-1]->pos[1];

    if (minElemSize[0] != 0 && minElemSize[1] != 0) {
        elemSize[0] = minElemSize[0];
        elemSize[1] = minElemSize[1];
    } else {
        std::cerr << "You must use the label #MPM_REGULAR_MESH_DETAILS with regular meshes." << std::endl;
    }

}