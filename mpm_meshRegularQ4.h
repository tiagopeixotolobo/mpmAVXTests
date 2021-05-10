#pragma once

#include "mpm_mesh.h"

namespace mpm {

    class mpmMeshRegularQ4: public mpmMesh {

    public:

        using mpmMesh::mpmMesh;

        double elemSize[2];

        int mapParticle2Element(mpmParticle*) override;
        void initializeMeshVariables() override;

    };

}