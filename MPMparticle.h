#pragma once

#include "Eigen/Core"

class mpmParticle
{
public:
	mpmParticle(int, int, double, double, double, double, double, double, double, int);
	virtual ~mpmParticle();

	int idMat;
	int idPart;
	int idBody;
	int idElem;

	double initialVolume;
	double kineticEnergyDensity;
	Eigen::Vector3d position;
	Eigen::Vector3d velocity;
	Eigen::Vector3d acceleration;
	Eigen::Vector3d initialPosition;

	double mass;

	Eigen::Vector3d traction;
	Eigen::Vector3d weight;

	Eigen::Matrix3d stress;
	Eigen::Matrix3d strain;
	Eigen::Matrix3d elastic_strain;
	double equivalent_plastic_strain;
	double mises_stress;
	double density;
	double pressure;

	Eigen::Matrix3d F;
	Eigen::Matrix3d L;
	Eigen::Matrix3d W;
	Eigen::Matrix3d D;

	// Herschel-Bulkley attributes
	double gamma_rate;
	double gamma_rate_max;
	double xi;
	double su;

	// Intact undrained shear strength (set by layers)
	double su0;

	double volume;
	double diameter;

	//Parallel
	int graph_color;
	int parallel_proc_idx;

	void setParticleStress(double[6]);
	void setParticleStrain(double[6]);
	void setParticleElasticStrain(double[6]);
	void setParticleDeformationGradient(double[9]);
	void setParticleEquivalentPlasticStrain(double);
	void setParticleMisesStress(double);
	//void calculateParticlesInitialMass(mpmMaterials* mat);
	void resetVelocityGradient();
	//void updateVelocityGradient(meshElement*, int, const Eigen::VectorXd&);
	void updateKinematics(double);
	void updateVolume();
	virtual bool isGhostParticle() { return false; }

};

