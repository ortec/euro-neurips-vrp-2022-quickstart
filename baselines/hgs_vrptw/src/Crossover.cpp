#include "Crossover.h"

Individual* Crossover::OX(Individual* parent1, Individual* parent2)
{
	// Picking the beginning and end of the crossover zone
	int start = params->rng() % params->nbClients;
	int end = params->rng() % params->nbClients;

	while (end == start) end = params->rng() % params->nbClients;

	// Frequency table to track the customers which have been already inserted
	std::vector <bool> freqClient = std::vector <bool>(params->nbClients + 1, false);

	Individual* result = new Individual(params, false);

	// Copy in place the elements from start to end (possibly "wrapping around" the end of the array)
	int j = start;
	while (j % params->nbClients != (end + 1) % params->nbClients)
	{
		result->chromT[j % params->nbClients] = parent1->chromT[j % params->nbClients];
		freqClient[result->chromT[j % params->nbClients]] = true;
		j++;
	}

	// Fill the remaining elements in the order given by the second parent
	for (int i = 1; i <= params->nbClients; i++)
	{
		int temp = parent2->chromT[(end + i) % params->nbClients];
		if (freqClient[temp] == false)
		{
			result->chromT[j % params->nbClients] = temp;
			j++;
		}
	}

	// Completing the individual with the Split algorithm
	split->generalSplit(result, params->nbVehicles);

	return result;
}

std::pair<Individual*, Individual*> Crossover::SREX(Individual* parent1, Individual* parent2)
{

	int nOfRoutesA = parent1->myCostSol.nbRoutes;
	int nOfRoutesB = parent2->myCostSol.nbRoutes;

	// Picking the beginning and end index of routes to replace of parent A
	// We like to replace routes with a large overlap of tasks, so we choose adjacent routes (they are sorted on polar angle)
	int startA = params->rng() % nOfRoutesA;
	int nOfMovedRoutes = params->rng() % (std::min(nOfRoutesA - 1, nOfRoutesB - 1)) + 1; // Prevent not moving any routes

	int startB = startA < nOfRoutesB ? startA : 0;

	std::unordered_set<int> clientsInSelectedA;
	for (int r = 0; r < nOfMovedRoutes; r++)
	{
		clientsInSelectedA.insert(parent1->chromR[(startA + r) % nOfRoutesA].begin(),
			parent1->chromR[(startA + r) % nOfRoutesA].end());
	}

	std::unordered_set<int> clientsInSelectedB;
	for (int r = 0; r < nOfMovedRoutes; r++)
	{
		clientsInSelectedB.insert(parent2->chromR[(startB + r) % nOfRoutesB].begin(),
			parent2->chromR[(startB + r) % nOfRoutesB].end());
	}

	bool improved = true;
	while (improved)
	{
		// Difference for moving 'left' in parent A
		const int differenceALeft = static_cast<int>(std::count_if(parent1->chromR[(startA - 1 + nOfRoutesA) % nOfRoutesA].begin(),
			parent1->chromR[(startA - 1 + nOfRoutesA) % nOfRoutesA].end(),
			[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }))
			- static_cast<int>(std::count_if(parent1->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA].begin(),
				parent1->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA].end(),
				[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }));

		// Difference for moving 'right' in parent A
		const int differenceARight = static_cast<int>(std::count_if(parent1->chromR[(startA + nOfMovedRoutes) % nOfRoutesA].begin(),
			parent1->chromR[(startA + nOfMovedRoutes) % nOfRoutesA].end(),
			[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }))
			- static_cast<int>(std::count_if(parent1->chromR[startA].begin(),
				parent1->chromR[startA].end(),
				[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }));

		// Difference for moving 'left' in parent B
		const int differenceBLeft = static_cast<int>(std::count_if(parent2->chromR[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].begin(),
			parent2->chromR[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].end(),
			[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }))
			- static_cast<int>(std::count_if(parent2->chromR[(startB - 1 + nOfRoutesB) % nOfRoutesB].begin(),
				parent2->chromR[(startB - 1 + nOfRoutesB) % nOfRoutesB].end(),
				[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }));

		// Difference for moving 'right' in parent B
		const int differenceBRight = static_cast<int>(std::count_if(parent2->chromR[startB].begin(),
			parent2->chromR[startB].end(),
			[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }))
			- static_cast<int>(std::count_if(parent2->chromR[(startB + nOfMovedRoutes) % nOfRoutesB].begin(),
				parent2->chromR[(startB + nOfMovedRoutes) % nOfRoutesB].end(),
				[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }));

		const int bestDifference = std::min({ differenceALeft, differenceARight, differenceBLeft, differenceBRight });

		if (bestDifference < 0)
		{
			if (bestDifference == differenceALeft)
			{
				for (int c : parent1->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
				{
					clientsInSelectedA.erase(clientsInSelectedA.find(c));
				}
				startA = (startA - 1 + nOfRoutesA) % nOfRoutesA;
				for (int c : parent1->chromR[startA])
				{
					clientsInSelectedA.insert(c);
				}
			}
			else if (bestDifference == differenceARight)
			{
				for (int c : parent1->chromR[startA])
				{
					clientsInSelectedA.erase(clientsInSelectedA.find(c));
				}
				startA = (startA + 1) % nOfRoutesA;
				for (int c : parent1->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
				{
					clientsInSelectedA.insert(c);
				}
			}
			else if (bestDifference == differenceBLeft)
			{
				for (int c : parent2->chromR[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
				{
					clientsInSelectedB.erase(clientsInSelectedB.find(c));
				}
				startB = (startB - 1 + nOfRoutesB) % nOfRoutesB;
				for (int c : parent2->chromR[startB])
				{
					clientsInSelectedB.insert(c);
				}
			}
			else if (bestDifference == differenceBRight)
			{
				for (int c : parent2->chromR[startB])
				{
					clientsInSelectedB.erase(clientsInSelectedB.find(c));
				}
				startB = (startB + 1) % nOfRoutesB;
				for (int c : parent2->chromR[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
				{
					clientsInSelectedB.insert(c);
				}
			}
		}
		else
		{
			improved = false;
		}
	}

	// Identify differences between route sets
	std::unordered_set<int> clientsInSelectedANotB;
	std::copy_if(clientsInSelectedA.begin(), clientsInSelectedA.end(),
		std::inserter(clientsInSelectedANotB, clientsInSelectedANotB.end()),
		[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); });

	std::unordered_set<int> clientsInSelectedBNotA;
	std::copy_if(clientsInSelectedB.begin(), clientsInSelectedB.end(),
		std::inserter(clientsInSelectedBNotA, clientsInSelectedBNotA.end()),
		[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) == clientsInSelectedA.end(); });

	Individual* offspring1 = new Individual(params, false);
	Individual* offspring2 = new Individual(params, false);

	// Replace selected routes from parent A with routes from parent B
	for (int r = 0; r < nOfMovedRoutes; r++)
	{
		int indexA = (startA + r) % nOfRoutesA;
		int indexB = (startB + r) % nOfRoutesB;
		offspring1->chromR[indexA].clear();
		offspring2->chromR[indexA].clear();

		for (int c : parent2->chromR[indexB])
		{
			offspring1->chromR[indexA].push_back(c);
			if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
			{
				offspring2->chromR[indexA].push_back(c);
			}
		}
	}

	// Move routes from parent A that are kept
	for (int r = nOfMovedRoutes; r < nOfRoutesA; r++)
	{
		int indexA = (startA + r) % nOfRoutesA;
		offspring1->chromR[indexA].clear();
		offspring2->chromR[indexA].clear();

		for (int c : parent1->chromR[indexA])
		{
			if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
			{
				offspring1->chromR[indexA].push_back(c);
			}
			offspring2->chromR[indexA].push_back(c);
		}
	}

	// Delete any remaining routes that still lived in offspring
	for (int r = nOfRoutesA; r < params->nbVehicles; r++)
	{
		offspring1->chromR[r].clear();
		offspring2->chromR[r].clear();
	}

	// Step 3: Insert unplanned clients (those that were in the removed routes of A but not the inserted routes of B)
	insertUnplannedTasks(offspring1, clientsInSelectedANotB);
	insertUnplannedTasks(offspring2, clientsInSelectedANotB);

	offspring1->evaluateCompleteCost();
	offspring2->evaluateCompleteCost();

	return std::make_pair(offspring1, offspring2);
}

void Crossover::insertUnplannedTasks(Individual* offspring, std::unordered_set<int> unplannedTasks)
{
	int newDistanceToInsert = INT_MAX;
	int newDistanceFromInsert = INT_MAX;
	int distanceDelta = INT_MAX;
	for (int c : unplannedTasks)
	{
		int earliestArrival = params->cli[c].earliestArrival;
		int latestArrival = params->cli[c].latestArrival;

		int bestDistance = INT_MAX;
		std::pair<int, int> bestLocation;

		for (int r = 0; r < params->nbVehicles; r++)
		{
			if (offspring->chromR[r].empty())
			{
				continue;
			}

			newDistanceFromInsert = params->timeCost.get(c, offspring->chromR[r][0]);
			if (earliestArrival + newDistanceFromInsert < params->cli[offspring->chromR[r][0]].latestArrival)
			{
				distanceDelta = params->timeCost.get(0, c) + newDistanceToInsert
					- params->timeCost.get(0, offspring->chromR[r][0]);
				if (distanceDelta < bestDistance)
				{
					bestDistance = distanceDelta;
					bestLocation = { r, 0 };
				}
			}

			for (int i = 1; i < static_cast<int>(offspring->chromR[r].size()); i++)
			{
				newDistanceToInsert = params->timeCost.get(offspring->chromR[r][i - 1], c);
				newDistanceFromInsert = params->timeCost.get(c, offspring->chromR[r][i]);
				if (params->cli[offspring->chromR[r][i - 1]].earliestArrival + newDistanceToInsert < latestArrival
					&& earliestArrival + newDistanceFromInsert < params->cli[offspring->chromR[r][i]].latestArrival)
				{
					distanceDelta = newDistanceToInsert + newDistanceFromInsert
						- params->timeCost.get(offspring->chromR[r][i - 1], offspring->chromR[r][i]);
					if (distanceDelta < bestDistance)
					{
						bestDistance = distanceDelta;
						bestLocation = { r, i };
					}
				}
			}

			newDistanceToInsert = params->timeCost.get(offspring->chromR[r].back(), c);
			if (params->cli[offspring->chromR[r].back()].earliestArrival + newDistanceToInsert < latestArrival)
			{
				distanceDelta = newDistanceToInsert + params->timeCost.get(c, 0)
					- params->timeCost.get(offspring->chromR[r].back(), 0);
				if (distanceDelta < bestDistance)
				{
					bestDistance = distanceDelta;
					bestLocation = { r, static_cast<int>(offspring->chromR[r].size()) };
				}
			}
		}

		offspring->chromR[bestLocation.first].insert(
			offspring->chromR[bestLocation.first].begin() + bestLocation.second, c);
	}
}

Crossover::Crossover(Params* params, Split* split) : params(params), split(split)
{
	
}

Crossover::~Crossover(void)
{
	
}
