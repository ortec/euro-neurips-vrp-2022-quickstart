/*MIT License

Original HGS-CVRP code: Copyright(c) 2020 Thibaut Vidal
Additional contributions: Copyright(c) 2022 ORTEC

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

#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include <vector>
#include <set>

#include "CircleSector.h"
#include "Individual.h"
#include "Params.h"

struct Node;

// Structure containing characterizations for a sequence
struct TimeWindowData
{
	int firstNodeIndex;
	int lastNodeIndex;
	int duration;         // Cumulative duration, including waiting and servicing
	int timeWarp;         // Cumulative time warp
	int earliestArrival;  // Earliest start of servicing first node in sequence, given a min cost route sequence
	int latestArrival;    // Latest start of servicing first node in sequence, given a min cost route sequence
	int latestReleaseTime;      // Latest of all release times of customers in sequence, so route cannot dispatch before

	// Note: [earliestArrival, latestArrival] represent the time in which we can arrive at the first node and execute
	// the min cost route. Arriving later would lead to (additional) time warp and arriving earlier would lead to
	// (additional) waiting time, not necessarily at the first node.
};

// Structure containing a route
struct Route
{
	int cour;							// Route index
	int nbCustomers;					// Number of customers visited in the route
	int whenLastModified;				// "When" this route has been last modified
	int whenLastTestedLargeNb;			// "When" the large neighborhood moves for this route have been last tested
	bool isDeltaRemovalTWOutdated;		// Flag to indicate deltaRemovalTW data of nodes is outdated
	Node* depot;						// Pointer to the associated depot
	int duration;						// Total time on the route (driving duration + service duration, excluding waiting time)
	int load;							// Total load on the route
	int reversalDistance;				// Difference of cost if the route is reversed
	TimeWindowData twData;				// Time window data of the route
	double penalty;						// Current sum of load, duration and time window penalties
	double polarAngleBarycenter;		// Polar angle of the barycenter of the route
	CircleSector sector;				// Circle sector associated to the set of clients
};

struct Node
{
	bool isDepot;						// Tells whether this node represents a depot or not
	int cour;							// Node index
	int position;						// Position in the route
	int whenLastTestedRI;				// "When" the RI moves for this node have been last tested
	Node* next;							// Next node in the route order
	Node* prev;							// Previous node in the route order
	Route* route;						// Pointer towards the associated route
	int cumulatedLoad;					// Cumulated load on this route until the client (including itself)
	int cumulatedTime;					// Cumulated time on this route until the client (including itself)
	int cumulatedReversalDistance;		// Difference of cost if the segment of route (0...cour) is reversed (useful for 2-opt moves with asymmetric problems)
	int deltaRemoval;					// Difference of cost in the current route if the node is removed (used in SWAP*)
	int deltaRemovalTW;					// Difference of cost in the current route if the node is removed, including TimeWarp (used in SWAP*)
	TimeWindowData twData;				// TimeWindowData for individual node (cour)
	TimeWindowData prefixTwData;		// TimeWindowData for subsequence (0...cour) including self
	TimeWindowData postfixTwData;		// TimeWindowData for subsequence (cour...0) including self
	bool isSeed;						// Tells whether a nextSeed is available (faster twData calculations)
	TimeWindowData toNextSeedTwD;		// TimeWindowData for subsequence (cour...cour+4) excluding self, including cour + 4
	Node* nextSeed;						// next seeded node if available (nullptr otherwise)
};

// This struct is used for grouping all node data that is relevant during the construction heuristics.
struct NodeToInsert
{
	int clientIdx; // Index in the clients vector
	int load; // Load of the node.
	int serviceDuration; // Service duration of the Node
	double angleFromDepot; // The angle of the node relative to the depot (between -PI and PI)
	TimeWindowData twData; // TimeWindowData for the Node
};

// Structure used in SWAP* to remember the three best insertion positions of a client in a given route
struct ThreeBestInsert
{
	int whenLastCalculated;
	int bestCost[3];
	Node* bestLocation[3];

	void compareAndAdd(int costInsert, Node* placeInsert)
	{
		if (costInsert >= bestCost[2]) return;
		else if (costInsert >= bestCost[1])
		{
			bestCost[2] = costInsert; bestLocation[2] = placeInsert;
		}
		else if (costInsert >= bestCost[0])
		{
			bestCost[2] = bestCost[1]; bestLocation[2] = bestLocation[1];
			bestCost[1] = costInsert; bestLocation[1] = placeInsert;
		}
		else
		{
			bestCost[2] = bestCost[1]; bestLocation[2] = bestLocation[1];
			bestCost[1] = bestCost[0]; bestLocation[1] = bestLocation[0];
			bestCost[0] = costInsert; bestLocation[0] = placeInsert;
		}
	}

	// Resets the structure (no insertion calculated)
	void reset()
	{
		bestCost[0] = INT_MAX; bestLocation[0] = nullptr;
		bestCost[1] = INT_MAX; bestLocation[1] = nullptr;
		bestCost[2] = INT_MAX; bestLocation[2] = nullptr;
	}

	ThreeBestInsert() { reset(); };
};

// Structured used to keep track of the best SWAP* move
struct SwapStarElement
{
	double moveCost = 1.e30;
	double loadPenU = 1.e30;
	double loadPenV = 1.e30;
	Node* U = nullptr;
	Node* bestPositionU = nullptr;
	Node* V = nullptr;
	Node* bestPositionV = nullptr;
};

// Main local learch structure
class LocalSearch
{

private:

	Params* params;							// Problem parameters
	bool searchCompleted;						// Tells whether all moves have been evaluated without success
	int nbMoves;								// Total number of moves (RI and SWAP*) applied during the local search. Attention: this is not only a simple counter, it is also used to avoid repeating move evaluations
	std::vector < int > orderNodes;				// Randomized order for checking the nodes in the RI local search
	std::vector < int > orderRoutes;			// Randomized order for checking the routes in the SWAP* local search
	std::set < int > emptyRoutes;				// indices of all empty routes
	int loopID;									// Current loop index

	/* THE SOLUTION IS REPRESENTED AS A LINKED LIST OF ELEMENTS */
	std::vector < Node > clients;				// Elements representing clients (clients[0] is a sentinel and should not be accessed)
	std::vector < Node > depots;				// Elements representing depots
	std::vector < Node > depotsEnd;				// Duplicate of the depots to mark the end of the routes
	std::vector < Route > routes;				// Elements representing routes
	std::vector<bool> bestInsertInitializedForRoute;
	std::vector < std::vector < ThreeBestInsert > > bestInsertClient;   // (SWAP*) For each route and node, storing the cheapest insertion cost (excluding TW)
	std::vector < std::vector < ThreeBestInsert > > bestInsertClientTW;   // (SWAP*) For each route and node, storing the cheapest insertion cost (including TW)

	/* TEMPORARY VARIABLES USED IN THE LOCAL SEARCH LOOPS */
	// nodeUPrev -> nodeU -> nodeX -> nodeXNext
	// nodeVPrev -> nodeV -> nodeY -> nodeYNext
	Node* nodeU;
	Node* nodeX;
	Node* nodeV;
	Node* nodeY;
	Route* routeU;
	Route* routeV;
	int nodeUPrevIndex, nodeUIndex, nodeXIndex, nodeXNextIndex;
	int nodeVPrevIndex, nodeVIndex, nodeYIndex, nodeYNextIndex;
	int loadU, loadX, loadV, loadY;
	int serviceU, serviceX, serviceV, serviceY;
	bool routeUTimeWarp, routeULoadPenalty, routeVTimeWarp, routeVLoadPenalty;
	double penaltyCapacityLS, penaltyTimeWarpLS;

	void setLocalVariablesRouteU(); // Initializes some local variables and distances associated to routeU to avoid always querying the same values in the distance matrix
	void setLocalVariablesRouteV(); // Initializes some local variables and distances associated to routeV to avoid always querying the same values in the distance matrix

	// Functions in charge of excess load penalty calculations
	inline double penaltyExcessLoad(double myLoad) { return std::max(0., myLoad - params->vehicleCapacity) * penaltyCapacityLS; }
	inline double penaltyTimeWindows(const TimeWindowData& twData) { 
		return (twData.timeWarp + std::max(twData.latestReleaseTime - twData.latestArrival, 0)) * penaltyTimeWarpLS; 
	}
	inline double deltaPenaltyTimeWindows(const TimeWindowData& twDataAdd, const TimeWindowData& twDataSubtract) { 
		return penaltyTimeWindows(twDataAdd) - penaltyTimeWindows(twDataSubtract); 
	}

	/* RELOCATE MOVES */
	// (Legacy notations: move1...move9 from Prins 2004)
	bool MoveSingleClient(); // If U is a client node, remove U and insert it after V
	bool MoveTwoClients(); // If U and X are client nodes, remove them and insert (U,X) after V
	bool MoveTwoClientsReversed(); // If U and X are client nodes, remove them and insert (X,U) after V

	/* SWAP MOVES */
	bool SwapTwoSingleClients(); // If U and V are client nodes, swap U and V
	bool SwapTwoClientsForOne(); // If U, X and V are client nodes, swap (U,X) and V
	bool SwapTwoClientPairs(); // If (U,X) and (V,Y) are client nodes, swap (U,X) and (V,Y)

	/* 2-OPT and 2-OPT* MOVES */
	bool TwoOptWithinTrip(); // If route(U) == route(V), replace (U,X) and (V,Y) by (U,V) and (X,Y)
	bool TwoOptBetweenTrips(); // If route(U) != route(V), replace (U,X) and (V,Y) by (U,Y) and (V,X)

	/*TW based operators*/
	bool ReorderTWsIfNeeded(); // For the current route try to order all TWs

	/* SUB-ROUTINES FOR EFFICIENT SWAP* EVALUATIONS */
	bool swapStar(bool withTW); // Calculates all SWAP* between routeU and routeV and apply the best improving move
	int getCheapestInsertSimultRemoval(Node* U, Node* V, Node*& bestPosition); // Calculates the insertion cost and position in the route of V, where V is omitted
	int getCheapestInsertSimultRemovalWithTW(Node* U, Node* V, Node*& bestPosition); // Calculates the insertion cost and position in the route of V, where V is omitted
	void preprocessInsertions(Route* R1, Route* R2); // Preprocess all insertion costs of nodes of route R1 in route R2
	void preprocessInsertionsWithTW(Route* R1, Route* R2); // Preprocess all insertion costs of nodes of route R1 in route R2

	/* RELOCATE MOVES BETWEEN TRIPS*/
	bool RelocateStar(); // Calculates all SWAP* between nodeU and all routes recently changed

	/* SUB-ROUTINES FOR TIME WINDOWS */
	TimeWindowData getEdgeTwData(Node* U, Node* V); // Calculates time window data for edge between U and V, does not have to be currently adjacent
	TimeWindowData getRouteSegmentTwData(Node* U, Node* V); // Calculates time window data for segment in single route
	TimeWindowData MergeTWDataRecursive(const TimeWindowData& twData1, const TimeWindowData& twData2);

	template <typename... Args>
	inline TimeWindowData MergeTWDataRecursive(const TimeWindowData& first, const TimeWindowData& second, Args... args)
	{
		TimeWindowData result = MergeTWDataRecursive(first, second);
		return MergeTWDataRecursive(result, args...);
	}

	/* ROUTINES TO UPDATE THE SOLUTIONS */
	static void insertNode(Node* U, Node* V);		// Solution update: Insert U after V
	static void swapNode(Node* U, Node* V);		// Solution update: Swap U and V
	void updateRouteData(Route* myRoute);			// Updates the preprocessed data of a route
	CostSol getCostSol(bool usePenaltiesLS = true);	// Computes cost and penalties for solutions, only used for additional (debug) checks

public:

	// Run the local search with the specified penalty values
	void run(Individual* indiv, double penaltyCapacityLS, double penaltyTimeWarpLS);

	// Initialize data for construction heuristics
	void initializeConstruction(Individual* indiv, std::vector<NodeToInsert>* nodesToInsert);

	// Construct an individual using a heuristic with a maximum allowed violation of capacity and a
	// maximum allowed time warp. If not all clients can be assigned given the tolerances, the
	// unassigned clients are added to the back of the last route (should not happen in practice,
	// since there are plenty routes). Routes are created sequentially. The seed client in the
	// route can be either the unassigned client furthest from depot, or the unassigned client
	// closest to the depot.
	void constructIndividualWithSeedOrder(int toleratedCapacityViolation, int toleratedTimeWarp,
		bool useSeedClientFurthestFromDepot, Individual* indiv);

	// Groups orders per route according to angle with depot. fillPercentage can be configured to
	// allow some room for repairing routes during local search. Orders with short time window are
	// added in order of time latestArrival, other orders are inserted in best position.
	void constructIndividualBySweep(int fillPercentage, Individual* indiv);

	// Loading an initial solution into the local search
	void loadIndividual(Individual* indiv);

	// Exporting the LS solution into an individual and calculating the penalized cost according to the original penalty weights from Params
	void exportIndividual(Individual* indiv);

	// Constructor
	LocalSearch(Params* params);
};

#endif
