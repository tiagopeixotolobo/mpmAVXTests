#include "mpm_node.h"

node::~node()
{
}

void node::resetNode() {
	this->mass = 0;

	velocity.setZero(); externalForce.setZero(); internalForce.setZero(); dashpotForce.setZero(); dampingForce.setZero();
	contactForce.setZero(); momentum.setZero(); normal.setZero(); totalForce.setZero(); stress.setZero();
	this->active = false; this->isInternal = false;
	extrapolatedPosition.setZero();

}

void node :: resetMassVelocityMomentum() {
	this->mass = 0;
	velocity.setZero(); 
	momentum.setZero();
}

void node::setSupport() {
	if (this->active){
		if (this->support[0]) {
			this->internalForce(0) = 0;
			this->externalForce(0) = 0;
			this->dashpotForce(0) = 0;
			this->dampingForce(0) = 0;
			this->momentum(0) = 0;
			this->velocity(0) = 0;
			this->totalForce(0) = 0;
			this->contactForce(0) = 0;
		}
		if (this->support[1]) {
			this->internalForce(1) = 0;
			this->externalForce(1) = 0;
            this->dashpotForce(1) = 0;
			this->dampingForce(1) = 0;
			this->momentum(1) = 0;
			this->velocity(1) = 0;
			this->totalForce(1) = 0;
			this->contactForce(1) = 0;
		}
	}
}