#include "meshes/mpm_meshGeneral.h"


using namespace mpm;

void mpmMeshGeneral::generateMappingMesh() {

    // We start from the most internal mesh and go until we reach the most external mesh.

    // First, we need to determine the elemSize of said mesh:

    subMesh = new mappingMesh(0);

    subMesh->elementSize = maxElemSize[0] > maxElemSize[1] ? maxElemSize[0] : maxElemSize[1];

    // Then, we determine the limits of the mesh:

    subMesh->nx = ceil((this->xLim[1] - this->xLim[0])/subMesh->elementSize);
    subMesh->ny = ceil((this->yLim[1] - this->yLim[0])/subMesh->elementSize);

    subMesh->xLim[0] = this->xLim[0];
    subMesh->yLim[0] = this->yLim[0];
    subMesh->xLim[1] = this->xLim[0] + subMesh->nx * subMesh->elementSize;
    subMesh->yLim[1] = this->yLim[0] + subMesh->ny * subMesh->elementSize;

    // We proceed to the creation of the nodes in the mesh:
    double n_x, n_y, n_z;

    for (unsigned int j = 0; j < subMesh->ny+1; j++) {

        for (unsigned int i = 0; i < subMesh->nx+1; i++) {

            n_x = subMesh->xLim[0] + i * subMesh->elementSize;
            n_y = subMesh->yLim[0] + j * subMesh->elementSize;
            n_z = 0;
            subMesh->nodes.push_back(new mappingNode(j * (subMesh->nx+1) + i + 1, n_x, n_y, n_z));

        }

    }

    // After that, we create the elements:

    mappingNode* nodes[4];

    for (unsigned int j = 0; j < subMesh->ny; j++) {

        for (unsigned int i = 0; i < subMesh->nx; i++) {

            nodes[0] = subMesh->nodes[j * (subMesh->nx+1) + i];
            nodes[1] = subMesh->nodes[j * (subMesh->nx+1) + i + 1];
            nodes[2] = subMesh->nodes[(j+1) * (subMesh->nx+1) + i + 1];
            nodes[3] = subMesh->nodes[(j+1) * (subMesh->nx+1) + i];

            subMesh->elements.push_back(new mappingElement(j * subMesh->nx + i + 1, nodes));

        }

    }

    // Then, we assign the original elements to the mapping mesh:

    for (unsigned int i = 0; i < this->body[0]->elements.size(); i++) {

        for (unsigned int j = 0; j < subMesh->elements.size(); j++) {

            if (inPolygon(this->body[0]->elements[i], subMesh->elements[j])) {
                subMesh->elements[j]->meshElements.push_back(i);
            }

        }
    }

}

void mpmMeshGeneral::computeMinAndMaxElemSizes() {

    double partial;
    minElemSize[0] = this->body[0]->elements[0]->xLimits[1] - this->body[0]->elements[0]->xLimits[0];
    minElemSize[1] = this->body[0]->elements[0]->yLimits[1] - this->body[0]->elements[0]->yLimits[0];
    maxElemSize[0] = this->body[0]->elements[0]->xLimits[1] - this->body[0]->elements[0]->xLimits[0];
    maxElemSize[1] = this->body[0]->elements[0]->yLimits[1] - this->body[0]->elements[0]->yLimits[0];

    for (int i=1; i<this->nElem; i++) {

        partial = this->body[0]->elements[i]->xLimits[1] - this->body[0]->elements[i]->xLimits[0];

        if (partial < minElemSize[0])
            minElemSize[0] = partial;

        partial = this->body[0]->elements[i]->yLimits[1] - this->body[0]->elements[i]->yLimits[0];

        if (partial < minElemSize[1])
            minElemSize[1] = partial;

        partial = this->body[0]->elements[i]->xLimits[1] - this->body[0]->elements[i]->xLimits[0];

        if (partial > maxElemSize[0])
            maxElemSize[0] = partial;

        partial = this->body[0]->elements[i]->yLimits[1] - this->body[0]->elements[i]->yLimits[0];

        if (partial > maxElemSize[1])
            maxElemSize[1] = partial;

    }

}

void mpmMeshGeneral::initializeMeshVariables() {

    nElem = body[0]->elements.size();

    double partial;

    xLim[0] = body[0]->nodes[0]->pos[0];
    xLim[1] = body[0]->nodes[0]->pos[0];
    yLim[0] = body[0]->nodes[0]->pos[1];
    yLim[1] = body[0]->nodes[0]->pos[1];

    for (int i = 1; i < body[0]->nodes.size(); i++) {

        partial = body[0]->nodes[i]->pos[0];

        if (partial < xLim[0])
            xLim[0] = partial;

        if (partial > xLim[1])
            xLim[1] = partial;

        partial = body[0]->nodes[i]->pos[1];

        if (partial < yLim[0])
            yLim[0] = partial;

        if (partial > yLim[1])
            yLim[1] = partial;

    }

    computeMinAndMaxElemSizes();
    generateMappingMesh();

}

bool mpmMeshGeneral::inPolygon(meshElement* containee, mappingElement* container) {

    int nvertexes = 4;
    bool _internal = true;

    double x, y, xr, yr, ymin, ymax, xb, xa;

    // Compute the coordinates of the baricentrum:
    x = (containee->nodes[0]->pos[0] + containee->nodes[1]->pos[0] + containee->nodes[2]->pos[0])/3.;
    y = (containee->nodes[0]->pos[1] + containee->nodes[1]->pos[1] + containee->nodes[2]->pos[1])/3.;

    int N = 0, j;
    yr = y;  // Horizontal line

    for(int i=0; i<nvertexes; i++){
        j = i+1;
        if (j > nvertexes-1)
            j = 0;
        if (container->nodes[i]->pos[1] != container->nodes[j]->pos[1]){
            if (container->nodes[i]->pos[1] > container->nodes[j]->pos[1]){
                ymin = container->nodes[j]->pos[1];
                ymax = container->nodes[i]->pos[1];
                xa = container->nodes[j]->pos[0];
                xb = container->nodes[i]->pos[0];
            }
            else{
                ymin = container->nodes[i]->pos[1];
                ymax = container->nodes[j]->pos[1];
                xa = container->nodes[i]->pos[0];
                xb = container->nodes[j]->pos[0];
            }

            if ( yr > ymin && yr <= ymax){
                xr = (y - ymin)*((xb-xa)/(ymax-ymin)) + xa;
                if (xr>x)
                    N = N+1;
            }
            else
                continue;
        }
    }
    if (N % 2 == 0)
        _internal = false;

    return _internal;

}

bool mpmMeshGeneral::inPolygon(mpmParticle* part, meshElement* elem) {

    int nvertexes = (int) elem->nodes.size();
    bool _internal = true;

    double x, y, xr, yr, ymin, ymax, xb, xa;

    x = part->position(0);
    y = part->position(1);

    int N = 0, j;
    yr = y;  // Horizontal line

    for(int i=0; i<nvertexes; i++){
        j = i+1;
        if (j > nvertexes-1)
            j = 0;
        if (elem->nodes[i]->pos[1] != elem->nodes[j]->pos[1]){
            if (elem->nodes[i]->pos[1] > elem->nodes[j]->pos[1]){
                ymin = elem->nodes[j]->pos[1];
                ymax = elem->nodes[i]->pos[1];
                xa = elem->nodes[j]->pos[0];
                xb = elem->nodes[i]->pos[0];
            }
            else{
                ymin = elem->nodes[i]->pos[1];
                ymax = elem->nodes[j]->pos[1];
                xa = elem->nodes[i]->pos[0];
                xb = elem->nodes[j]->pos[0];
            }

            if ( yr > ymin && yr <= ymax){
                xr = (y - ymin)*((xb-xa)/(ymax-ymin)) + xa;
                if (xr>x)
                    N = N+1;
            }
            else
                continue;
        }
    }
    if (N % 2 == 0)
        _internal = false;

    return _internal;

}

int mpmMeshGeneral::mapParticle2Element(mpmParticle* part) {

    double idPart[2];

    // FIRST STEP: Mapping to the regular mesh:

    idPart[0] = (part->position[0] - subMesh->xLim[0]) / subMesh->elementSize;
    idPart[0] = ceil(idPart[0]);
    idPart[1] = (part->position[1] - subMesh->yLim[0]) / subMesh->elementSize;
    idPart[1] = ceil(idPart[1]);
    int idElem = (int)((idPart[1] - 1) * (subMesh->nx) + idPart[0] - 1); // This is the vector index, not the element id.

    // SECOND STEP: Mapping to the neighbors of the element in the regular mesh.

    int candidate;

    for ( int k = -1; k < 2; k++) {

        for (int j = -1; j < 2; j++) {

            candidate = k * subMesh->nx + idElem + j;

            if (candidate >= 0 && candidate < subMesh->elements.size()) {

                for (unsigned int i = 0; i < subMesh->elements[candidate]->meshElements.size(); i++) {

                    if (inPolygon(part, this->body[0]->elements[subMesh->elements[candidate]->meshElements[i]])) {
                        idElem = subMesh->elements[candidate]->meshElements[i];
                        break;
                    }

                }

            }

        }


    }

    return idElem;

}