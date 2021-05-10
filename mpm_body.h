#pragma once

#include "mpm_node.h"
#include "mpm_meshelement.h"
#include "meshElementL2.h"
#include <vector>

namespace mpm {

    class mpmBody
    {
    public:
        mpmBody(int, std::string);
        ~mpmBody();

        int meshID;
        int bodyID;
        std::string bodyLabel;
        bool hasPrescribedVelocity;
        std::vector<node*> nodes;
        std::vector<meshElement*> elements;
        std::vector<meshElementL2*> boundaryElements;

    };

}