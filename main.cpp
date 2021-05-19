#include "engine.h"
#include <chrono>

int main() {

	std::chrono::time_point<std::chrono::system_clock> start, end;
	std::chrono::duration<double> elapsed_seconds, meanTime;

	int nExecutions = 10;
	int nDivisions = 5;

	double elemSize0 = 0.021;
	double dx0 = elemSize0/4;
	double dy0 = dx0;

	int nParticles, nNodes;

	for (auto i = 0; i < nDivisions; i++) //numero de divisoes
	{
		meanTime.zero();

		for (auto j = 0; j < nExecutions; j++) //numero de execucoes
		{
			engine* engineAVX = new engine();

			start = std::chrono::system_clock::now();

			engineAVX->startAVXData(elemSize0, dx0, dy0);

			end = std::chrono::system_clock::now();
			elapsed_seconds = end - start;

			meanTime += elapsed_seconds;

			nParticles = engineAVX->particles.size();
			nNodes = engineAVX->backgroundMesh->body[0]->nodes.size();

			delete engineAVX;
		}

		std::cout << "N Particles: " << nParticles << "\t\t" << "N Nodes: " << nNodes << std::endl;
		std::cout << "Time Average: " << meanTime.count()/nExecutions << std::endl;
		std::cout << "----------------------------------------------------------------" << std::endl;

		elemSize0 *= 0.7;
		dx0 = elemSize0/4;
		dy0 = dx0;
	}

	return 0;
}