#pragma once

constexpr int nNodesMaxNodesList = 16;

// Mathematical constants:
#define MPM_PI 3.14159265358979323846

// Mesh definitions:
#define MESH_GENERAL 0
#define MESH_REGULAR_Q4 2

// Simulation types:
#define SIMULATION_1D 1
#define SIMULATION_2D 2
#define SIMULATION_3D 3

// Element types:
#define ELEMENT_NONE 0
#define ELEMENT_Q4 1
#define ELEMENT_T3 2
#define ELEMENT_L2 3

// Material types:
#define MATERIAL_SOLID 0
#define MATERIAL_LINEAR_ELASTIC 1
#define MATERIAL_ELASTOPLASTIC_VONMISES 2
#define MATERIAL_ELASTOPLASTIC_DRUCKERPRAGER 3
#define MATERIAL_VISCOPLASTIC_HERSCHELBULKLEY 4
#define MATERIAL_IDEAL_FLUID 5

// Layer types:
#define LAYER_GEOLAYER 0

// Integration schemes:
#define USF_INTEGRATION 1
#define USL_INTEGRATION 2
#define MUSL_INTEGRATION 3

// Velocity functions:
#define CONSTANT_VELOCITY_FUNCTION 1
#define PIECEWISE_LINEAR_VELOCITY_FUNCTION 2
#define SINE_VELOCITY_FUNCTION 3
#define COSINE_VELOCITY_FUNCTION 4

// Xmdf labels
#define XMDF_MASS "mass"
#define XMDF_BODY "Body"
#define XMDF_VOLUME "volume"
#define XMDF_ACCELERATION "acceleration"
#define XMDF_VELOCITY "velocity"
#define XMDF_DISPLACEMENT "displacement"
#define XMDF_STRESS "stress"
#define XMDF_MISES "mises"
#define XMDF_STRAIN "strain"
#define XMDF_EPS "eqpstrain"
#define XMDF_SOFTENING_RATIO "softening_ratio"
#define XMDF_TRACTION "traction"
#define XMDF_PRESSURE "pressure"
#define XMDF_GRAPH_COLOR "graph_color"
#define XMDF_PARTITION_INDEX "partition_idx"
#define XMDF_SU "su"