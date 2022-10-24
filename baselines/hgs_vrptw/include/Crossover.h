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

#ifndef CROSSOVER_H
#define CROSSOVER_H

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include "Individual.h"
#include "Split.h"

class Crossover
{
private:

	Params* params;				// Problem parameters
	Split* split;					// Split algorithm

	// SREX Crossover
	void insertUnplannedTasks(Individual* offspring, std::unordered_set<int> unplanned);	

public:

	Individual* OX(Individual* parent1, Individual* parent2);
	std::pair<Individual*, Individual*> SREX(Individual* parent1, Individual* parent2);

	// Constructor
	Crossover(Params* params, Split* split);

	// Destructor
	~Crossover(void);
};

#endif
