#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_set>
#include <map>
#include "Eigen/Core"

#include "MPMparticle.h"
#include "mpm_mesh.h"
#include "mpm_meshGeneral.h"
#include "mpm_meshRegularQ4.h"
//#include "mpm_materials.h"
//#include "materials/mpm_linearElastic.h"
//#include "materials/mpm_vonMises.h"
//#include "materials/mpm_druckerPrager.h"
//#include "materials/mpm_herschelBulkley.h"
//#include "layers/mpm_layer.h"
//#include "mpm_prescribedVelocity.h"
//#include "velocity_functions/mpm_constantVelocityFunction.h"
//#include "velocity_functions/mpm_piecewiseLinearVelocityFunction.h"
//#include "velocity_functions/mpm_sineVelocityLabel.h"
//#include "velocity_functions/mpm_cosineVelocityFunction.h"


namespace mpm {
	class mpmData
	{
	public:
		///Constructor
		mpmData();
		///Destructor
		~mpmData();

		//Input filename
		std::string input_filename;

		bool usingGIMP;

		//Time settings
		double t;
		double t0;
		double tf;
		double dt;
		double dtCritical;
		double dtCritFraction;
		int numPrintSamples;
		bool adaptiveTimeStep;

		//Contact settings
		double frictionCoefficient;
		double frictionalBound;
		int contactPenaltyFactorPowerIndex;
		bool contactPenaltyFactorDistanceToNode;
		bool allowSeparationAfterContact;
		bool maximumShearStressAlgorithm;
		bool maximumShearStressAlgorithmSpecifiedNodes;
		bool setInternalNodes;

		//Contact flags
		bool contactStandard;
		bool contactNairn;
		bool contactMa;

		//Particle integrator
		int xPicOrder;
		bool usingXpic;
		bool usingSecondOrderParticleIntegrator;

		//Physical behavior settings
		double localDamping;
		double picDamping;
		double massTolerance;
		double kineticEnergyDensity;
        bool saturatedSoil;
		bool staticProblem;
		bool energyRestricted;
		double energyLowerBoundary;
		bool useFbarAlgorithm;

		//Output settings
		bool isPrintingMPMFile;
		bool isPrintingXMDFFile;
        bool isPrintingReportsOnPrintSteps;
		bool mpmPrintFinalStepOnly;
		bool using_demview;
		int outputPrecision;

		//simulation type
		int simType;

		//Integration Type
		int integrationScheme;

		//mesh type
		int meshType;
		int nBodies;

		//step counter
		unsigned int step;
		
		//xmdf labels
		std::unordered_set<std::string> xmdfLabels;

		//Gravity
		Eigen::Vector3d gravity;
        
        //Parallel
        bool benchmark_mode;
        int num_benchmark_steps;
        int num_threads;
		int parallel_num_processes;
		int parallel_proc_idx;

		////// VARIABLES THAT ARE NOT INITIALIZED IN THE CONSTRUCTOR //////

		//maximum velocities
		double maxVelocity[2];

		//maximum SoundSpeed
		double maxSoundSpeed;

		//Particles and BackgroundMesh
		std::vector<mpmParticle*> particles;
		mpmMesh* backgroundMesh = nullptr;
		//std::map<int, mpmMaterials *> materials;
		//std::vector<mpmLayer*> layers;
		//std::vector<prescribedVelocityFunction*> velocityFunctions;
		std::vector<std::string> bodyLabels;
		std::unordered_set<std::string> xmdfBodyLabels;
			   
	};
}
