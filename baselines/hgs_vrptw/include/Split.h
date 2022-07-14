/*MIT License

Copyright(c) 2020 Thibaut Vidal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef SPLIT_H
#define SPLIT_H

#include <vector>
#include <utility>

#include "Params.h"
#include "Individual.h"

// Structure representing a client used in the Split algorithm
struct ClientSplit
{
	int demand;				// The demand of the client
	int serviceTime;		// The service duration of the client
	int d0_x;				// The distance from the depot to the client
	int dx_0;				// The distance from the client to the depot
	int dnext;				// The distance from the client to the next client

	// Constructor, initializing everything with zero
	ClientSplit() : demand(0), serviceTime(0), d0_x(0), dx_0(0), dnext(0) {}
};

// Simple Deque which is used for all Linear Split algorithms
struct Trivial_Deque
{
	std::vector<int> myDeque;	// Vector structure to keep the elements of the queue
	int indexFront;				// Index of the front element
	int indexBack;				// Index of the back element
	
	// Removes the front element of the queue
	inline void pop_front()
	{ 
		indexFront++;
	}

	// Removes the back element of the queue D
	inline void pop_back()
	{
		indexBack--;
	}

	// Appends a new element to the back of the queue D
	inline void push_back(int i)
	{
		indexBack++;
		myDeque[indexBack] = i;
	} 

	// Returns the front element of the queue
	inline int get_front()
	{
		return myDeque[indexFront];
	}

	// Returns the second-front element of the queue
	inline int get_next_front()
	{
		return myDeque[indexFront + 1];
	}

	// Returns the back element of the queue
	inline int get_back()
	{
		return myDeque[indexBack];
	}

	// Resets the queue
	void reset(int firstNode)
	{
		myDeque[0] = firstNode;
		indexBack = 0;
		indexFront = 0;
	}

	// Returns the size of the queue
	inline int size()
	{
		return indexBack - indexFront + 1;
	}

	// Constructor, to creata a queue with place for nbElements elements, where firstNode is the first node
	Trivial_Deque(int nbElements, int firstNode)
	{
		myDeque = std::vector<int>(nbElements);
		myDeque[0] = firstNode;
		indexBack = 0;
		indexFront = 0;
	}
};

class Split
{
private:
	Params* params;			// Problem parameters
	int maxVehicles;		// Maximum number of vehicles (not lower than the trivial (LP) Bin Packing Bound)

	// Auxiliary data structures to run the Linear Split algorithm (all of size nbClients + 1)
	std::vector<ClientSplit> cliSplit;					// Vector of all clientSplits (size nbClients + 1, but nothing stored for the depot!)
	std::vector<std::vector<double>> potential;			// potential[0][t] is the costs of a shortest path from 0 to t (so we want to minimize the potential)
	// The next variable pred stores the client starting the route of a given client. So pred[k] is the client starting the route where k is also in.
	std::vector<std::vector<int>> pred;					// Indice of the predecessor in an optimal path
	std::vector<int> sumDistance;						// Cumulative distance. sumDistance[i] for i > 1 contains the sum of distances : sum_{k=1}^{i-1} d_{k,k+1}
	std::vector<int> sumLoad;							// Cumulative demand. sumLoad[i] for i >= 1 contains the sum of loads : sum_{k=1}^{i} q_k
	std::vector<int> sumService;						// Cumulative service time. sumService[i] for i >= 1 contains the sum of service time : sum_{k=1}^{i} s_k

	// To be called with i < j only
	// Computes the cost of propagating the label i until j
	inline double propagate(int i, int j, int k)
	{
		return potential[k][i] + sumDistance[j] - sumDistance[i + 1] + cliSplit[i + 1].d0_x + cliSplit[j].dx_0
			+ params->penaltyCapacity * std::max(sumLoad[j] - sumLoad[i] - params->vehicleCapacity, 0);
	}

	// Tests if i dominates j as a predecessor for all nodes x >= j+1
	// We assume that i < j
	inline bool dominates(int i, int j, int k)
	{
		return potential[k][j] + cliSplit[j + 1].d0_x > potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1] - sumDistance[i + 1]
			+ params->penaltyCapacity * (sumLoad[j] - sumLoad[i]);
	}

	// Tests if j dominates i as a predecessor for all nodes x >= j+1
	// We assume that i < j
	inline bool dominatesRight(int i, int j, int k)
	{
		return potential[k][j] + cliSplit[j + 1].d0_x < potential[k][i] + cliSplit[i + 1].d0_x + sumDistance[j + 1] - sumDistance[i + 1] + MY_EPSILON;
	}

	// Split for unlimited fleet
	int splitSimple(Individual* indiv);

	// Split for limited fleet
	int splitLF(Individual* indiv);

public:
	// General Split function (tests the unlimited fleet, and only if it does not produce a feasible solution, runs the Split algorithm for limited fleet)
	void generalSplit(Individual* indiv, int nbMaxVehicles);

	// Constructor
	Split(Params* params);
};

#endif
