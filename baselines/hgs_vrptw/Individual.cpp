#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include "Individual.h"
#include "Params.h"

void Individual::evaluateCompleteCost()
{
	// Create an object to store all information regarding solution costs
	myCostSol = CostSol();
	// Loop over all routes that are not empty
	for (int r = 0; r < params->nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			int latestReleaseTime = params->cli[chromR[r][0]].releaseTime;
			for (int i = 1; i < static_cast<int>(chromR[r].size()); i++)
			{
				latestReleaseTime = std::max(latestReleaseTime, params->cli[chromR[r][i]].releaseTime);
			}
			// Get the distance, load, serviceDuration and time associated with the vehicle traveling from the depot to the first client
			// Assume depot has service time 0 and earliestArrival 0
			int distance = params->timeCost.get(0, chromR[r][0]);
			int load = params->cli[chromR[r][0]].demand;
			int service = params->cli[chromR[r][0]].serviceDuration;
			// Running time excludes service of current node. This is the time that runs with the vehicle traveling
			// We start the route at the latest release time (or later but then we can just wait and there is no penalty for waiting)
			int time = latestReleaseTime + distance;
			int waitTime = 0;
			int timeWarp = 0;
			// Add possible waiting time
			if (time < params->cli[chromR[r][0]].earliestArrival)
			{
				// Don't add wait time since we can start route later 
				// (doesn't really matter since there is no penalty anyway)
				// waitTime += params->cli[chromR[r][0]].earliestArrival - time;
				time = params->cli[chromR[r][0]].earliestArrival;
			}
			// Add possible time warp
			else if (time > params->cli[chromR[r][0]].latestArrival)
			{
				timeWarp += time - params->cli[chromR[r][0]].latestArrival;
				time = params->cli[chromR[r][0]].latestArrival;
			}
			predecessors[chromR[r][0]] = 0;

			// Loop over all clients for this vehicle
			for (int i = 1; i < static_cast<int>(chromR[r].size()); i++)
			{
				// Sum the distance, load, serviceDuration and time associated with the vehicle traveling from the depot to the next client
				distance += params->timeCost.get(chromR[r][i - 1], chromR[r][i]);
				load += params->cli[chromR[r][i]].demand;
				service += params->cli[chromR[r][i]].serviceDuration;
				time = time + params->cli[chromR[r][i - 1]].serviceDuration + params->timeCost.get(chromR[r][i - 1], chromR[r][i]);

				// Add possible waiting time
				if (time < params->cli[chromR[r][i]].earliestArrival)
				{
					waitTime += params->cli[chromR[r][i]].earliestArrival - time;
					time = params->cli[chromR[r][i]].earliestArrival;
				}
				// Add possible time warp
				else if (time > params->cli[chromR[r][i]].latestArrival)
				{
					timeWarp += time - params->cli[chromR[r][i]].latestArrival;
					time = params->cli[chromR[r][i]].latestArrival;
				}
				
				// Update predecessors and successors
				predecessors[chromR[r][i]] = chromR[r][i - 1];
				successors[chromR[r][i - 1]] = chromR[r][i];
			}

			// For the last client, the successors is the depot. Also update the distance and time
			successors[chromR[r][chromR[r].size() - 1]] = 0;
			distance += params->timeCost.get(chromR[r][chromR[r].size() - 1], 0);
			time = time + params->cli[chromR[r][chromR[r].size() - 1]].serviceDuration + params->timeCost.get(chromR[r][chromR[r].size() - 1], 0);
			
			// For the depot, we only need to check the end of the time window (add possible time warp)
			if (time > params->cli[0].latestArrival)
			{
				timeWarp += time - params->cli[0].latestArrival;
				time = params->cli[0].latestArrival;
			}
			// Update variables that track stats on the whole solution (all vehicles combined)
			myCostSol.distance += distance;
			myCostSol.waitTime += waitTime;
			myCostSol.timeWarp += timeWarp;
			myCostSol.nbRoutes++;
			if (load > params->vehicleCapacity)
			{
				myCostSol.capacityExcess += load - params->vehicleCapacity;
			}
		}
	}

	// When all vehicles are dealt with, calculated total penalized cost and check if the solution is feasible. (Wait time does not affect feasibility)
	myCostSol.penalizedCost = myCostSol.distance + myCostSol.capacityExcess * params->penaltyCapacity + myCostSol.timeWarp * params->penaltyTimeWarp + myCostSol.waitTime * params->penaltyWaitTime;
	isFeasible = (myCostSol.capacityExcess < MY_EPSILON && myCostSol.timeWarp < MY_EPSILON);
}

void Individual::shuffleChromT()
{
	// Initialize the chromT with values from 1 to nbClients
	for (int i = 0; i < params->nbClients; i++)
	{
		chromT[i] = i + 1;
	}
	// Do a random shuffle chromT from begin to end
	std::shuffle(chromT.begin(), chromT.end(), params->rng);
}

void Individual::removeProximity(Individual* indiv)
{
	// Get the first individual in indivsPerProximity
	auto it = indivsPerProximity.begin();
	// Loop over all individuals in indivsPerProximity until indiv is found
	while (it->second != indiv)
	{
		++it;
	}
	// Remove indiv from indivsPerProximity
	indivsPerProximity.erase(it);
}

double Individual::brokenPairsDistance(Individual* indiv2)
{
	// Initialize the difference to zero. Then loop over all clients of this individual
	int differences = 0;
	for (int j = 1; j <= params->nbClients; j++)
	{
		// Increase the difference if the successor of j in this individual is not directly linked to j in indiv2
		if (successors[j] != indiv2->successors[j] && successors[j] != indiv2->predecessors[j])
		{
			differences++;
		}
		// Last loop covers all but the first arc. Increase the difference if the predecessor of j in this individual is not directly linked to j in indiv2
		if (predecessors[j] == 0 && indiv2->predecessors[j] != 0 && indiv2->successors[j] != 0)
		{
			differences++;
		}
	}
	return static_cast<double>(differences) / params->nbClients;
}

double Individual::averageBrokenPairsDistanceClosest(int nbClosest)
{
	double result = 0;
	int maxSize = std::min(nbClosest, static_cast<int>(indivsPerProximity.size()));
	auto it = indivsPerProximity.begin();
	for (int i = 0; i < maxSize; i++)
	{
		result += it->first;
		++it;
	}
	return result / maxSize;
}

void Individual::exportCVRPLibFormat(std::string fileName)
{
	std::cout << "----- WRITING SOLUTION WITH VALUE " << myCostSol.penalizedCost << " IN : " << fileName << std::endl;
	std::ofstream myfile(fileName);
	if (myfile.is_open())
	{
		for (int k = 0; k < params->nbVehicles; k++)
		{
			if (!chromR[k].empty())
			{
				myfile << "Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
				for (int i : chromR[k])
				{
					myfile << " " << i;
				}
				myfile << std::endl;
			}
		}
		myfile << "Cost " << (int) myCostSol.penalizedCost << std::endl;
		myfile << "Time " << params->getTimeElapsedSeconds() << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
}

void Individual::printCVRPLibFormat()
{
	std::cout << "----- PRINTING SOLUTION WITH VALUE " << myCostSol.penalizedCost << std::endl;
	for (int k = 0; k < params->nbVehicles; k++)
	{
		if (!chromR[k].empty())
		{
			std::cout << "Route #" << k + 1 << ":"; // Route IDs start at 1 in the file format
			for (int i : chromR[k])
			{
				std::cout << " " << i;
			}
			std::cout << std::endl;
		}
	}
	std::cout << "Cost " << (int) myCostSol.penalizedCost << std::endl;
	std::cout << "Time " << params->getTimeElapsedSeconds() << std::endl;
	fflush(stdout);
}

bool Individual::readCVRPLibFormat(std::string fileName, std::vector<std::vector<int>>& readSolution, double& readCost)
{
	readSolution.clear();
	std::ifstream inputFile(fileName);
	if (inputFile.is_open())
	{
		std::string inputString;
		inputFile >> inputString;
		// Loops as long as the first line keyword is "Route"
		for (int r = 0; inputString == "Route"; r++)
		{
			readSolution.push_back(std::vector<int>());
			inputFile >> inputString;
			getline(inputFile, inputString);
			std::stringstream ss(inputString);
			int inputCustomer;
			// Loops as long as there is an integer to read
			while (ss >> inputCustomer)
			{
				readSolution[r].push_back(inputCustomer);
			}
			inputFile >> inputString;
		}
		if (inputString == "Cost")
		{
			inputFile >> readCost;
			return true;
		}
		else std::cout << "----- UNEXPECTED WORD IN SOLUTION FORMAT: " << inputString << std::endl;
	}
	else std::cout << "----- IMPOSSIBLE TO OPEN: " << fileName << std::endl;
	return false;
}

Individual::Individual(Params* params, bool initializeChromTAndShuffle) : params(params), isFeasible(false), biasedFitness(0)
{
	successors = std::vector<int>(params->nbClients + 1);
	predecessors = std::vector<int>(params->nbClients + 1);
	chromR = std::vector<std::vector<int>>(params->nbVehicles);
	chromT = std::vector<int>(params->nbClients);
	if (initializeChromTAndShuffle)
	{
		shuffleChromT();
	}
}

Individual::Individual(Params* params, std::string solutionStr) : params(params), isFeasible(false), biasedFitness(0)
{
	successors = std::vector<int>(params->nbClients + 1);
	predecessors = std::vector<int>(params->nbClients + 1);
	chromR = std::vector<std::vector<int>>(params->nbVehicles);
	chromT = std::vector<int>(params->nbClients);

	std::stringstream ss(solutionStr);
	int inputCustomer;
	// Loops as long as there is an integer to read
	int pos = 0;
	int route = 0;
	while (ss >> inputCustomer)
	{
		if (inputCustomer == 0)
		{
			// Depot
			route++;
			assert(route < params->nbVehicles);
		}
		else
		{
			chromR[route].push_back(inputCustomer);
			chromT[pos] = inputCustomer;
			pos++;
		}
	}
	assert (pos == params->nbClients);
	evaluateCompleteCost();
}

Individual::Individual(): params(nullptr), isFeasible(false), biasedFitness(0)
{
	myCostSol.penalizedCost = 1.e30;
}
