#include "mpm_mappingMesh.h"


using namespace mpm;

mappingNode::mappingNode(int id, double x, double y, double z) {

    ID = id;
    pos[0] = x; pos[1] = y; pos[2] = z;

}

mappingElement::mappingElement(int id, mpm::mappingNode **nd) {

    ID = id;
    nodes[0] = nd[0];
    nodes[1] = nd[1];
    nodes[2] = nd[2];
    nodes[3] = nd[3];

}
