#include "engine.h"

engine::engine() {}

engine::~engine() {
	
	for (auto p : particles) {
		delete p;
	}
	particles.clear();

	if (backgroundMesh != nullptr) {
		delete backgroundMesh;
	}
}

void engine::startAVXData(double elemSize0, double dx0, double dy0) {

	setDummyParameters(0, 3, -6.0, 2.0, elemSize0, 0.3, dx0, dy0); //x0mesh, xfmesh, y0mesh, yfmesh, elemsize, side, x0, y0
	startDummyMesh();
	createParticles();
	mapParticles2Elements();
	mapParticles2Nodes();
}

void engine::setDummyParameters(double x0mesh, double xfmesh, double y0mesh, double yfmesh, double elemsize, double side, double dx, double dy) {

	this->avxData.x0mesh = x0mesh - elemsize;
	this->avxData.xfmesh = xfmesh + elemsize;
	this->avxData.y0mesh = y0mesh - elemsize;
	this->avxData.yfmesh = yfmesh + elemsize;
	this->avxData.xMeshAmplitude = this->avxData.xfmesh - this->avxData.x0mesh;
	this->avxData.yMeshAmplitude = this->avxData.yfmesh - this->avxData.y0mesh;
	this->avxData.backgroundMeshElemsize = elemsize;
	this->avxData.nElements = std::ceil((xfmesh - x0mesh) / elemsize) * std::ceil((yfmesh - y0mesh) / elemsize);
	this->avxData.bodyLabels = { "0" };

	this->avxData.gravity = { 0, -9.81, 0 };
	this->avxData.density = 1000.0;

	this->avxData.side = side;
	this->avxData.x0 = (3 - this->avxData.side) / 2;
	this->avxData.y0 = 0.5;
	this->avxData.xf = this->avxData.x0 + this->avxData.side;
	this->avxData.yf = this->avxData.y0 + this->avxData.side;
	this->avxData.dx = dx;
	this->avxData.dy = dy;
}

void engine::startDummyMesh()
{
	this->backgroundMesh = new mpm::mpmMeshRegularQ4(avxData.bodyLabels);

	createNodes();
	createElements();
	loadNodeList();
}

void engine::createNodes() {

	double xSum, ySum;
	xSum = this->avxData.x0mesh;
	ySum = this->avxData.y0mesh;

	for (auto i = 0; i <= std::ceil(avxData.yMeshAmplitude / avxData.backgroundMeshElemsize); i++) //y
	{
		for (auto j = 0; j <= std::ceil(avxData.xMeshAmplitude / avxData.backgroundMeshElemsize); j++) //x
		{
			int id = i * std::ceil(avxData.xMeshAmplitude / avxData.backgroundMeshElemsize + 1) + j;
			backgroundMesh->addNode(id, xSum, ySum, 0, 0, 0, 0);
			xSum += avxData.backgroundMeshElemsize;
		}

		xSum = avxData.x0mesh;
		ySum += avxData.backgroundMeshElemsize;
	}
}

void engine::createElements() {

	int idElem, nodesId[4];
	int j = 0;

	int temp = std::ceil(avxData.xMeshAmplitude / avxData.backgroundMeshElemsize);

	for (auto i = 0; i < avxData.nElements; i++)
	{
		if (i % temp == 0 && i > 0) {
			j++;
		}

		idElem = i;

		nodesId[0] = i + j;
		nodesId[1] = i + j + 1;
		nodesId[2] = i + j + 2 + std::ceil(avxData.xMeshAmplitude / avxData.backgroundMeshElemsize);
		nodesId[3] = i + j + 1 + std::ceil(avxData.xMeshAmplitude / avxData.backgroundMeshElemsize);

		backgroundMesh->addElemQ4Traditional(idElem, nodesId);
	}
}

void engine::createParticles() {

	int id = 0;

	for (auto yp = avxData.y0 + avxData.dy / 2; yp < avxData.yf; yp += avxData.dy) {
		for (auto xp = avxData.x0 + avxData.dx / 2; xp < avxData.xf; xp += avxData.dx) {

			mpmParticle* p = new mpmParticle(id, 0, avxData.dx * avxData.dy, xp, yp, 0, 0, 0, 0, 0);
			p->mass = avxData.density * p->volume;
			p->weight = p->mass * avxData.gravity;
			particles.push_back(p);
			id++;
		}
	}
}

void engine::mapParticles2Elements() {

	int i, j;

	for (auto k = 0; k < particles.size(); k++)
	{
		i = std::floor(std::abs(avxData.x0mesh - particles[k]->position(0)) / avxData.backgroundMeshElemsize);
		j = std::floor(std::abs(avxData.y0mesh - particles[k]->position(1)) / avxData.backgroundMeshElemsize);
		particles[k]->idElem = j * std::ceil(avxData.xMeshAmplitude / avxData.backgroundMeshElemsize) + i;
	}
}

void engine::loadNodeList() {

	for (auto i = 0; i < backgroundMesh->body.size(); i++) {
		for (auto j = 0; j < backgroundMesh->body[i]->elements.size(); j++) {
			for (auto k = 0; k < backgroundMesh->body[i]->elements[j]->nodes.size(); k++)
			{
				backgroundMesh->body[i]->elements[j]->nodeList.push_back(backgroundMesh->body[i]->elements[j]->nodes[k]);
			}
		}
	}
}

void engine::mapParticles2Nodes() {

	Eigen::Vector3d SigmadN;
	int bodyId, idElem;
	Eigen::VectorXd N(nNodesMaxNodesList), dN(2 * nNodesMaxNodesList);

	for (int i = 0; i < particles.size(); i++) {

		//Linearizing element position ID
		idElem = particles[i]->idElem;

		bodyId = particles[i]->idBody;

		//Form Function and its Derivative (body 0 is used)
		backgroundMesh->body[0]->elements[idElem]->calculateFormFunction(particles[i]->position, .5 * particles[i]->diameter, N);
		backgroundMesh->body[0]->elements[idElem]->calculateDerivateFormFunction(particles[i]->position, .5 * particles[i]->diameter, dN);

		//loop into nodes from idElem
		for (unsigned int k = 0; k < backgroundMesh->body[0]->elements[idElem]->nodeList.size(); k++) {

			//NODAL MASS
			backgroundMesh->body[0]->elements[idElem]->nodeList[k]->mass += N(k) * particles[i]->mass;
			backgroundMesh->body[bodyId]->elements[idElem]->nodeList[k]->mass += N(k) * particles[i]->mass;

			//NODAL POSITION
			backgroundMesh->body[0]->elements[idElem]->nodeList[k]->extrapolatedPosition += N(k) * particles[i]->mass * particles[i]->position;
			backgroundMesh->body[bodyId]->elements[idElem]->nodeList[k]->extrapolatedPosition += N(k) * particles[i]->mass * particles[i]->position;

			//NODAL MOMENTUM
			backgroundMesh->body[0]->elements[idElem]->nodeList[k]->momentum += N(k) * particles[i]->mass * particles[i]->velocity;
			backgroundMesh->body[bodyId]->elements[idElem]->nodeList[k]->momentum += N(k) * particles[i]->mass * particles[i]->velocity;

			//NODAL NORMAL
			SigmadN(0) = dN(k);
			SigmadN(1) = dN(k + backgroundMesh->body[bodyId]->elements[idElem]->nodeList.size());
			SigmadN(2) = 0;
			backgroundMesh->body[bodyId]->elements[idElem]->nodeList[k]->normal += particles[i]->mass * SigmadN;

			//CHECKING IF A NODE IS PART OF A RIGID OBJECT
			if (!backgroundMesh->body[bodyId]->hasPrescribedVelocity) {
				//NODAL STRESS
				backgroundMesh->body[bodyId]->elements[idElem]->nodeList[k]->stress += N(k) * particles[i]->stress;

				//NODAL EXTERNAL FORCES
				backgroundMesh->body[0]->elements[idElem]->nodeList[k]->externalForce += N(k) * (particles[i]->weight + particles[i]->traction);
				backgroundMesh->body[bodyId]->elements[idElem]->nodeList[k]->externalForce += N(k) * (particles[i]->weight + particles[i]->traction);

				//NODAL INTERNAL FORCES
				backgroundMesh->body[0]->elements[idElem]->nodeList[k]->internalForce -= particles[i]->volume * particles[i]->stress * SigmadN;
				backgroundMesh->body[bodyId]->elements[idElem]->nodeList[k]->internalForce -= particles[i]->volume * particles[i]->stress * SigmadN;
			}
		}
	}
}
