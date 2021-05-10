#pragma once
#include "Eigen/Core"
#include <vector>

class node
{
public:

	node(int nId, double px, double py, double pz, bool sx, bool sy, bool sz) {
		this->nodeId = nId; this->pos << px, py, pz; this->support[0] = sx; this->support[1] = sy; this->support[2] = sz;
		contactForce.setZero(); hasMassAboveThreshold = false; isGeoContactNode = false; isInternal = false; 
		extrapolatedPosition.setZero();
	};
	~node();

	int nodeId;
	Eigen::Vector3d pos;
	Eigen::Vector3d internalForce;
	Eigen::Vector3d externalForce;
	Eigen::Vector3d dashpotForce;
	Eigen::Vector3d dampingForce;
	Eigen::Vector3d contactForce;
	Eigen::Vector3d totalForce;
	Eigen::Vector3d velocity;
	double mass;
	Eigen::Vector3d momentum;
	Eigen::Vector3d normal;
	Eigen::Matrix3d stress;
	std::vector<int> IdParticle;
	Eigen::Vector3d extrapolatedPosition;

	bool support[3];

	bool isInternal;
	bool hasMassAboveThreshold;
	bool isGeoContactNode;

	bool active;

	void resetMassVelocityMomentum();
	void resetNode();
	void setSupport();
};

