#include "mpm_meshelement.h"

meshElement::meshElement(int id) {
	this->idElem = id;
	this->type = ELEMENT_NONE;
    this->isInternal = false;
}

meshElement::~meshElement() {

}

double meshElement::computeArea() {

    double area = this->xLimits[1] - this->xLimits[0];  // TO DO: verify this calculation - variable xLimits is not initialized.
    return area;

}

double meshElement::computeVolume() {

    // This method used the Gauss Area equation to compute
    // the area of any irregular polygon.
    // Requisite:
    // Nodes must be ordered clockwisely or counterclockwisely.

    double V = 0;
    unsigned int j;

    for (unsigned int i = 0; i < nodes.size(); i++) {

        if (i != nodes.size() - 1) {
            j = i + 1;
        } else {
            j = 0;
        }

        V += nodes[i]->pos[1] * nodes[j]->pos[0] - nodes[i]->pos[0] * nodes[j]->pos[1];

    }

    return abs(.5 * V);

}

void meshElement::computeLimits() {

    this->xLimits[0] = this->nodes[0]->pos[0];
    this->xLimits[1] = this->nodes[0]->pos[0];

    this->yLimits[0] = this->nodes[0]->pos[1];
    this->yLimits[1] = this->nodes[0]->pos[1];

    for (int i = 1; i < this->nodes.size(); i++) {

        if (this->nodes[i]->pos[0] < this->xLimits[0]) {
            this->xLimits[0] = nodes[i]->pos[0];
        } else if (this->nodes[i]->pos[0] > this->xLimits[1]) {
            this->xLimits[1] = nodes[i]->pos[0];
        }

        if (this->nodes[i]->pos[1] < this->yLimits[0]) {
            this->yLimits[0] = nodes[i]->pos[1];
        } else if (this->nodes[i]->pos[1] > this->yLimits[1]) {
            this->yLimits[1] = nodes[i]->pos[1];
        }

    }

}