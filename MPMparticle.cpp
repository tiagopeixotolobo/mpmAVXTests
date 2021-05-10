#include "MPMparticle.h"

#include "Eigen/LU"
#include <cmath>


mpmParticle::mpmParticle(int idParticle, int idMaterial, double initialV, double x0, double y0, double z0, double v0x, double v0y, double v0z, int bodyID)
{
	this->idPart = idParticle; this->idMat = idMaterial; this->volume = initialV; this->initialVolume = initialV;

	position << x0, y0, z0;
	initialPosition << x0, y0, z0;
	velocity << v0x, v0y, v0z;

	this->acceleration.setZero(); this->stress.setZero(); this->elastic_strain.setZero(); this->equivalent_plastic_strain = 0.; this->mises_stress = 0.;
	this->L.setZero(); this->W.setZero(); this->D.setZero(); this->F.setIdentity(); this->strain.setZero();
	this->gamma_rate = 0; this->gamma_rate_max = 0; this->xi = 0; this->su = 0.0;
	this->su0 = 0;

	this->traction.setZero();
	this->weight.setZero();

	this->diameter = sqrt(this->initialVolume);
	this->idBody = bodyID;
	this->kineticEnergyDensity = 0;

	this->idElem = 0;
	this->graph_color = 0;
	this->parallel_proc_idx = 0;
}

mpmParticle::~mpmParticle()
{
}

void mpmParticle::setParticleStress(double strs[6]) {

	this->stress(0) = strs[0];
	this->stress(1) = strs[3];
	this->stress(2) = strs[4];
	this->stress(3) = strs[3];
	this->stress(4) = strs[1];
	this->stress(5) = strs[5];
	this->stress(6) = strs[4];
	this->stress(7) = strs[5];
	this->stress(8) = strs[2];
}

void mpmParticle::setParticleStrain(double strn[6]) {

	this->strain(0) = strn[0];
	this->strain(1) = strn[3];
	this->strain(2) = strn[4];
	this->strain(3) = strn[3];
	this->strain(4) = strn[1];
	this->strain(5) = strn[5];
	this->strain(6) = strn[4];
	this->strain(7) = strn[5];
	this->strain(8) = strn[2];
}

void mpmParticle::setParticleElasticStrain(double el_strn[6]) {

	this->elastic_strain(0) = el_strn[0];
	this->elastic_strain(1) = el_strn[3];
	this->elastic_strain(2) = el_strn[4];
	this->elastic_strain(3) = el_strn[3];
	this->elastic_strain(4) = el_strn[1];
	this->elastic_strain(5) = el_strn[5];
	this->elastic_strain(6) = el_strn[4];
	this->elastic_strain(7) = el_strn[5];
	this->elastic_strain(8) = el_strn[2];

}

void mpmParticle::setParticleDeformationGradient(double defGradient[9]) {

	this->F(0) = defGradient[0];
	this->F(1) = defGradient[1];
	this->F(2) = defGradient[2];
	this->F(3) = defGradient[3];
	this->F(4) = defGradient[4];
	this->F(5) = defGradient[5];
	this->F(6) = defGradient[6];
	this->F(7) = defGradient[7];
	this->F(8) = defGradient[8];

}

void mpmParticle::setParticleEquivalentPlasticStrain(double peeq) {

	this->equivalent_plastic_strain = peeq;
}

void mpmParticle::setParticleMisesStress(double mises) {

	this->mises_stress = mises;
}


//void mpmParticle::calculateParticlesInitialMass(mpmMaterials* mat)
//{
//	this->density = mat->rho;
//	this->mass = this->density * this->initialVolume;
//}

void mpmParticle::resetVelocityGradient() {
	this->L.setZero();
}

//void mpmParticle::updateVelocityGradient(meshElement* meshEl, int j, const Eigen::VectorXd& dN) {
//
//	L(0, 0) += meshEl->nodeList[j]->velocity(0) * dN(j);
//	L(1, 0) += meshEl->nodeList[j]->velocity(1) * dN(j);
//	L(1, 1) += meshEl->nodeList[j]->velocity(1) * dN(j + meshEl->nodeList.size());
//	L(0, 1) += meshEl->nodeList[j]->velocity(0) * dN(j + meshEl->nodeList.size());
//}

void mpmParticle::updateKinematics(double dt) {
	// spin tensor and deformation rate
	W = 0.5 * (L - L.transpose());
	D = 0.5 * (L + L.transpose());

	// update deformation gradient
	Eigen::Matrix3d I; I.setIdentity();
	F = (I + L * dt) * F;
}

void mpmParticle::updateVolume() {
	this->volume = this->initialVolume * (1 + this->strain.trace());
	this->density = this->mass / this->volume;
}
