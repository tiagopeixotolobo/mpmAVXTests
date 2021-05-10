#pragma once

#include "mpm_mesh.h"
#include "mpm_mappingMesh.h"

namespace mpm {

    class mpmMeshGeneral: public mpmMesh {

    public:

        using mpmMesh::mpmMesh;

        double maxElemSize[2];

        int mapParticle2Element(mpmParticle*) override;
        void initializeMeshVariables() override;
        void computeMinAndMaxElemSizes();
        void generateMappingMesh();

    private:
        bool inPolygon(mpmParticle*, meshElement*);
        bool inPolygon(meshElement*, mappingElement*);
        mappingMesh* subMesh;

    };

}