#pragma once
#include <vector>
#include "mpm_meshelement.h"
#include "mpm_mesh.h"

namespace mpm {

    class mappingNode {

    public:
        mappingNode(int, double, double, double);
        ~mappingNode() {};

        int ID;
        double pos[3];

    };

    class mappingElement {

    public:
        mappingElement(int id, mappingNode* nd[4]);
        ~mappingElement() {};

        int ID;
        mappingNode* nodes[4];
        std::vector<int> meshElements;

    };

    class mappingMesh {

    public:
        mappingMesh(int id) { this->ID = id; };
        ~mappingMesh() {};

        int nx, ny;
        double elementSize, xLim[2], yLim[2];

        int ID;
        std::vector<mappingElement*> elements;
        std::vector<mappingNode*> nodes;

    };
}
