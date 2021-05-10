#include "mpm_mesh.h"


mpmMesh::mpmMesh(std::vector<std::string> bodyLabels)
{
	this->xLim[0] = -9999999; this->xLim[1] = 9999999; this->yLim[0] = -9999999; this->yLim[1] = 9999999;

    int nBodies = bodyLabels.size();

    // Addition of body 0 is performed separately, as it does not come with a label.
    mpm::mpmBody *m = new mpm::mpmBody(0, "0");
	body.push_back(m);

    // Addition of the remaining bodies is performed in the sequence:
	for (int i = 0; i < nBodies; i++) {
        mpm::mpmBody *m = new mpm::mpmBody(i+1, bodyLabels[i]);
	    body.push_back(m);
	}	
}

mpmMesh::~mpmMesh()
{
    for (auto p : body)
        delete p;
    body.clear();
}

void mpmMesh::resetGrid(){
    
	for (int i = 0; i < body.size(); ++i) {
        #pragma omp parallel for
		for (int j = 0; j < body[i]->nodes.size(); j++)
			body[i]->nodes[j]->resetNode();
        #pragma omp parallel for
        for (int j = 0; j < body[i]->elements.size(); j++) {
            body[i]->elements[j]->active = false;
            body[i]->elements[j]->volume = 0.0;
            body[i]->elements[j]->volumeCorrected = 0.0;
        }
	}
}

void mpmMesh::resetMassVelocityMomentum() {

	for (int i = 0; i < body.size(); ++i) {
		if (!body[i]->hasPrescribedVelocity) { //se3 n tiver vel presc
			for (int j = 0; j < this->body[i]->nodes.size(); ++j) {
				this->body[i]->nodes[j]->resetMassVelocityMomentum();
			}
		}
	}
}

void mpmMesh::addNode(int idNode, double x, double y, double z, bool sx, bool sy, bool sz) {

	for (int i = 0; i < body.size(); i++) {
		node *n = new node(idNode, x, y, z, sx, sy, sz);
		body[i]->nodes.push_back(n);
	}

}

void mpmMesh::addElemT3(int idElem, int nodes[3]) {

    for (int i = 0; i < body.size(); i++) {
        meshElementT3 *elem = new meshElementT3(idElem);

        elem->nodes.push_back(body[i]->nodes[nodes[0] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[1] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[2] - 1]);

        if (i == 0)
            elem->computeLimits();

        body[i]->elements.push_back(elem);
    }
}

void mpmMesh::addElemL2(int idElem, int nodes[2]) {

    for (int i = 0; i < body.size(); i++) {
        meshElementL2 *elem = new meshElementL2(idElem);

        elem->nodes.push_back(body[i]->nodes[nodes[0] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[1] - 1]);

        body[i]->boundaryElements.push_back(elem);
    }
}

void mpmMesh::addElemQ4Traditional(int idElem, int nodes[4]) {

    for (int i = 0; i < body.size(); i++) {
        meshElementQ4Traditional *elem = new meshElementQ4Traditional(idElem);

        elem->nodes.push_back(body[i]->nodes[nodes[0] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[1] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[2] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[3] - 1]);

        if (i == 0)
            elem->computeLimits();

        body[i]->elements.push_back(elem);
    }
}

void mpmMesh::addElemQ4GIMP(int idElem, int nodes[4]) {

    for (int i = 0; i < body.size(); i++) {
        meshElementQ4GIMP *elem = new meshElementQ4GIMP(idElem);

        elem->nodes.push_back(body[i]->nodes[nodes[0] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[1] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[2] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[3] - 1]);

        if (i == 0)
            elem->computeLimits();

        body[i]->elements.push_back(elem);
    }
}

void mpmMesh::addElemQ4General(int idElem, int nodes[4]) {

    for (int i = 0; i < body.size(); i++) {
        meshElementQ4General *elem = new meshElementQ4General(idElem);

        elem->nodes.push_back(body[i]->nodes[nodes[0] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[1] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[2] - 1]);
        elem->nodes.push_back(body[i]->nodes[nodes[3] - 1]);

        if (i == 0)
            elem->computeLimits();

        body[i]->elements.push_back(elem);
    }
}

void mpmMesh::addLowerDimensionElement(meshElement* baseElem, int ID, int nodes[2]) {

    // TO DO: Generalize this for each child of meshElement.
    switch (baseElem->type) {

        case ELEMENT_Q4: {
            this->addElemL2(ID, nodes);
            break;
        }
        default: {
            this->addElemL2(ID, nodes);
            break;
        }
    }

}

std::string mpmMesh::getBodyLabelFromID(int bodyId) {

    for (auto bd: body)
        if (bd->bodyID == bodyId)
            return bd->bodyLabel;

    return std::string();

}

int mpmMesh::getBodyIDFromLabel(const std::string& bodyLabel) {

    for (auto bd: body)
        if (bd->bodyLabel == bodyLabel)
            return bd->bodyID;

    return 0;

}