#include "mpm_body.h"

using namespace mpm;

mpmBody::mpmBody(int ID, std::string bodyLabel) {

    this->meshID = ID;
    this->bodyID = ID;
    this->bodyLabel = bodyLabel;
    this->hasPrescribedVelocity = false;
}

mpmBody::~mpmBody() {

    for (auto p : nodes)
        delete p;
    nodes.clear();

    for (auto p : elements)
        delete p;
    elements.clear();

    for (auto p : boundaryElements)
        delete p;
    boundaryElements.clear();

    // The velocity functions are deleted at data, as there could be more than one body
    // using the same velocity function.

}