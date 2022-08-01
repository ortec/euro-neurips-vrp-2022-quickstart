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

#ifndef PARAMS_H
#define PARAMS_H

#include <assert.h>
#include <string>
#include <vector>
#include <limits.h>
#include <iostream>
#include <ctime>
#include <chrono>

#include "Matrix.h"
#include "xorshift128.h"

#define MY_EPSILON 0.00001		// Precision parameter, used to avoid numerical instabilities
#define PI 3.14159265359		// Number pi, with 11 decimal precision

// Structure of a Client, including its index, position, and all other variables and parameters
struct Client
{
	int custNum;			// Index of the client
	int coordX;				// Coordinate X
	int coordY;				// Coordinate Y
	int serviceDuration;	// Service duration
	int demand;				// Demand
	int earliestArrival;	// Earliest arrival (when using time windows)
	int latestArrival;		// Latest arrival (when using time windows)
	int releaseTime;	// Release time (when using time windows, route containing this customer cannot depart before this time)
	int polarAngle;			// Polar angle of the client around the depot (starting at east, moving counter-clockwise), measured in degrees and truncated for convenience
};

// This is needed for the initialization of a Params variable
class CommandLine;

// Class that stores all the parameters (from the command line) (in Config) and data of the instance needed by the algorithm
class Params
{
public:
	// Stores all the parameters values (given by using the command line)
	struct Config
	{
		int nbIter = 20000;				// Number of iterations without improvement until termination. Default value: 20,000 iterations
		int timeLimit = INT_MAX;		// CPU time limit until termination in seconds. Default value: infinity
		bool useWallClockTime = false;  // If True, measure wall clock time rather than CPU time
		std::string pathBKS = "";		// Path to Best Known Solution

		// Parameters for the Construction Heuristics
		double fractionGeneratedNearest = 0.05;				// Proportion of individuals constructed by nearest-first
		double fractionGeneratedFurthest = 0.05;			// Proportion of individuals constructed by furthest-first
		double fractionGeneratedSweep = 0.05;				// Proportion of individuals constructed by sweep
		double fractionGeneratedRandomly = 0.85;			// Proportion of individuals constructed randomly
		int minSweepFillPercentage = 60;					// Fill rate in BKS is always more than 40%, so I don't think less than this would make sense.
															// The maximum vehicle usage is 40% (100/250 routes, see SINTEF BKS),
															// so take 60% to have some margin (otherwise all remaining orders will be in last route)
		int maxToleratedCapacityViolation = 50;				// In the instance I checked vehicle capacity was 1000, so max 5% could make sense.
		int maxToleratedTimeWarp = 100;						// No real feeling yet for what value makes sense.
		double initialTimeWarpPenalty = 1.0;				// This was the default until now, but with this value feasible individuals often
															// become infeasible during the local search in doLocalSearchAndAddIndividual.
															// With initialTimeWarpPenalty = 10, this does not happen.
		double penaltyBooster = 2.;							// Set to value > 0, so penalty will get multiplied by this value (instead of default 1.2) if num feasible == 0

		// Parameters of the Genetic Algorithm
		int minimumPopulationSize = 25;							// Minimum population size
		int generationSize = 40;								// Number of solutions created before reaching the maximum population size (i.e., generation size)
		int nbElite = 4;										// Number of elite individuals (reduced in HGS-2020)
		int nbClose = 5;										// Number of closest solutions/individuals considered when calculating diversity contribution
		double targetFeasible = 0.2;							// Reference proportion for the number of feasible individuals, used for the adaptation of the penalty parameters
		int repairProbability = 50;								// Integer (0-100) representing repair probability if individual is infeasible after local search
		int growNbGranularAfterNonImprovementIterations = 5000; // The number of iterations without improvements after which the nbGranular is grown
		int growNbGranularAfterIterations = 0;					// The number of iteration after which the nbGranular is grown
		int growNbGranularSize = 0;								// The number nbGranular is increase by
		int growPopulationAfterNonImprovementIterations = 5000; // The number of iterations without improvements after which the minimumPopulationSize is grown
		int growPopulationAfterIterations = 0;					// The number of iteration after which minimumPopulationSize is grown
		int growPopulationSize = 0;								// The number minimumPopulationSize is increase by
		double diversityWeight = 0.;							// Weight for diversity criterium, if 0, weight is 1 - nbElite / populationSize
		std::string initialSolution = "";						// Initial solution, represented as 'giant tour' with 0 for depot: 1 2 3 0 4 5 6

		// Other parameters
		int nbVeh = INT_MAX;								// Number of vehicles
		int logPoolInterval = 0;							// The verbose level of the algorithm log
		bool isDimacsRun = false;							// If DIMACS run, print incumbent and avoid other output
		bool useDynamicParameters = false;					// To use dynamic parameters based on instance attributes
		std::string pathSolution;							// Solution path
		int nbGranular = 40;								// Granular search parameter, limits the number of moves in the RI local search
		int intensificationProbabilityLS = 15;				// Probability intensification moves are performed during LS ([0-100])
		bool useSwapStarTW = true;							// Use TW swap star
		bool skipSwapStarDist = false;						// Skip normal swap star based on distance
		int circleSectorOverlapToleranceDegrees = 0;		// Margin to take (in degrees 0 - 359) to determine overlap of circle sectors for SWAP*
		int minCircleSectorSizeDegrees = 15;				// Minimum size (in degrees) for circle sectors such that even small circle sectors have 'overlap'
		int seed = 0;										// Random seed. Default value: 0
		std::string pathInstance;							// Instance path
		bool useSymmetricCorrelatedVertices = false;		// When correlation matrix is symmetric
		bool doRepeatUntilTimeLimit = true;					// When to repeat the algorithm when max nr of iter is reached, but time limit is not
	};

	Config config;						// Stores all the parameter values
	XorShift128 rng;					// Fast random number generator
	std::chrono::system_clock::time_point startWallClockTime;			// Start wall clock time of this object (should be constructed at start of program)
	std::clock_t startCPUTime;			// Start CPU time of this object

	// Adaptive penalty coefficients
	double penaltyCapacity;				// Penalty for one unit of capacity excess (adapted through the search)
	double penaltyWaitTime;				// Penalty for one unit waiting time (adapted through the search)
	double penaltyTimeWarp;				// Penalty for one unit time warp (adapted through the search)

	double proximityWeightWaitTime;		// Weight for waiting time in defining the neighbourhood proximities
	double proximityWeightTimeWarp;		// Weight for time warp in defining the neighbourhood proximities

	// Data of the problem instance
	std::string instanceName;
	bool isDurationConstraint;											// Indicates if the problem includes duration constraints
	bool isTimeWindowConstraint;										// Indicates if the problem includes time window constraints
	bool isExplicitDistanceMatrix;										// Indicates if the problem is with explicit distances (non-euclidean)
	int nbClients;														// Number of clients (excluding the depot)
	int nbVehicles;														// Number of vehicles
	int durationLimit;													// Route duration limit
	int vehicleCapacity;												// Capacity limit
	int totalDemand;													// Total demand required by the clients
	int maxDemand;														// Maximum demand of a client
	int maxDist;														// Maximum distance between two clients
	std::vector<Client> cli;											// Vector containing information on each client (including the depot!)
	Matrix timeCost;													// Distance matrix (including the depot!)
	std::vector<std::vector<std::pair<double, int>>> orderProximities;	// For each client, other clients sorted by proximity (size nbClients + 1, but nothing stored for the depot!)
	std::vector<std::vector<int>> correlatedVertices;					// Neighborhood restrictions: For each client, list of nearby clients (size nbClients + 1, but nothing stored for the depot!)
	int circleSectorOverlapTolerance;									// Tolerance when determining circle sector overlap (0 - 65536)
	int minCircleSectorSize;											// Minimum circle sector size to enforce (for nonempty routes) (0 - 65536)

	// Initialization from a given data set
	Params(const CommandLine&);

	// Get time elapsed since start of program
	double getTimeElapsedSeconds();

	// Whether time limit is exceeded
	bool isTimeLimitExceeded();

	// Calculate, for all vertices, the correlation for the nbGranular closest vertices
	void SetCorrelatedVertices();
};

#endif
