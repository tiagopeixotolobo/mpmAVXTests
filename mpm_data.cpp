#include "mpm_data.h"
#include <iostream>

namespace mpm {
	mpmData::mpmData(){
		input_filename = "";

        usingGIMP = false;

		//Time settings
		t = 0;
		t0 = 0;
		tf = 0;
		dtCritical = 0;
		dtCritFraction = 0;
		numPrintSamples = 0;
        adaptiveTimeStep = true;

		//Contact settings
		frictionCoefficient = 0;
		frictionalBound = 1e16;
		contactPenaltyFactorPowerIndex = 8;
		contactPenaltyFactorDistanceToNode = false;
		allowSeparationAfterContact = true;
		maximumShearStressAlgorithm = false;
		maximumShearStressAlgorithmSpecifiedNodes = false;
		setInternalNodes = false;

		// Contact flags
		contactStandard = false;
		contactNairn = true;
		contactMa = false;

		//Particle integrator
		xPicOrder = 1;
		usingXpic = false;
		usingSecondOrderParticleIntegrator = false;

		//Physical behavior settings
		localDamping = 0.0;
		picDamping = 0.0;
		massTolerance = 1e-20;
		kineticEnergyDensity = 0;
        saturatedSoil = false;	
		staticProblem = false;	
		energyRestricted = false;
		energyLowerBoundary = 0;
		useFbarAlgorithm = false;

		//Output settings
		isPrintingMPMFile = false;
		isPrintingXMDFFile = true;
        isPrintingReportsOnPrintSteps = false;
		mpmPrintFinalStepOnly = false;
		using_demview = true;
		outputPrecision = 6;

		//Simulation type
		simType = SIMULATION_2D;

        //Integration Scheme
        integrationScheme = MUSL_INTEGRATION;

		//Mesh type
		meshType = MESH_GENERAL;
		nBodies = -1;

		//Step counter
		step = 0;

		//xmdf Labels
		xmdfLabels = {XMDF_MASS, XMDF_BODY, XMDF_VOLUME, XMDF_DISPLACEMENT, XMDF_VELOCITY, XMDF_STRESS, XMDF_STRAIN};

		//gravity
		gravity.setZero();

        //Parallel
        benchmark_mode = false;
        num_benchmark_steps=-1;
        num_threads=0;
		parallel_num_processes=1;
		parallel_proc_idx=0;
      
	}


	mpmData::~mpmData()
	{
	    for (auto p: particles)
	        delete p;
	    particles.clear();

		if (backgroundMesh != nullptr)
			delete backgroundMesh;

	 //   for (auto p: materials)
	 //       delete p.second;
	 //   materials.clear();

		//for (auto L : layers)
		//	delete L;
		//layers.clear();

		//for (auto p: velocityFunctions)
		//	delete p;
		//velocityFunctions.clear();
	}
}
