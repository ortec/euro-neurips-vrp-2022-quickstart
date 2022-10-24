#include "Crossover.h"
#include "Genetic.h"
#include "Individual.h"
#include "LocalSearch.h"
#include "Params.h"
#include "Population.h"
#include "Split.h"

#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(hgspy, m)
{
    py::class_<Params::Config>(m, "Config")
        .def(py::init<int,
                    int,
                    bool,
                    std::string,
                    double,
                    double,
                    double,
                    double,
                    int,
                    int,
                    int,
                    double,
                    double,
                    int,
                    int,
                    int,
                    int,
                    double,
                    int,
                    int,
                    int,
                    int,
                    int,
                    int,
                    int,
                    double,
                    std::string,
                    int,
                    int,
                    bool,
                    bool,
                    std::string,
                    int,
                    int,
                    bool,
                    bool,
                    int,
                    int,
                    int,
                    std::string,
                    bool,
                    bool>(),
             py::arg("nbIter") = 20'000,
             py::arg("timeLimit") = INT_MAX,
             py::arg("useWallClockTime") = false,
             py::arg("pathBKS") = "",
             py::arg("fractionGeneratedNearest") = 0.05,
             py::arg("fractionGeneratedFurthest") = 0.05,
             py::arg("fractionGeneratedSweep") = 0.05,
             py::arg("fractionGeneratedRandomly") = 0.85,
             py::arg("minSweepFillPercentage") = 60,
             py::arg("maxToleratedCapacityViolation") = 50,
             py::arg("maxToleratedTimeWarp") = 100,
             py::arg("initialTimeWarpPenalty") = 1.0,
             py::arg("penaltyBooster") = 2.0,
             py::arg("minimumPopulationSize") = 25,
             py::arg("generationSize") = 40,
             py::arg("nbElite") = 4,
             py::arg("nbClose") = 5,
             py::arg("targetFeasible") = 0.2,
             py::arg("repairProbability") = 50,
             py::arg("growNbGranularAfterNonImprovementIterations") = 5'000,
             py::arg("growNbGranularAfterIterations") = 0,
             py::arg("growNbGranularSize") = 0,
             py::arg("growPopulationAfterNonImprovementIterations") = 5'000,
             py::arg("growPopulationAfterIterations") = 0,
             py::arg("growPopulationSize") = 0,
             py::arg("diversityWeight") = 0.,
             py::arg("initialSolution") = "",
             py::arg("nbVeh") = INT_MAX,
             py::arg("logPoolInterval") = 0,
             py::arg("isDimacsRun") = false,
             py::arg("useDynamicParameters") = false,
             py::arg("pathSolution") = "",
             py::arg("nbGranular") = 40,
             py::arg("intensificationProbabilityLS") = 15,
             py::arg("useSwapStarTW") = true,
             py::arg("skipSwapStarDist") = false,
             py::arg("circleSectorOverlapToleranceDegrees") = 0,
             py::arg("minCircleSectorSizeDegrees") = 15,
             py::arg("seed") = 0,
             py::arg("pathInstance") = "",
             py::arg("useSymmetricCorrelatedVertices") = false,
             py::arg("doRepeatUntilTimeLimit") = true)
        .def_readwrite("nbIter", &Params::Config::nbIter)
        .def_readwrite("timeLimit", &Params::Config::timeLimit)
        .def_readwrite("useWallClockTime", &Params::Config::useWallClockTime)
        .def_readwrite("pathBKS", &Params::Config::pathBKS)
        .def_readwrite("fractionGeneratedNearest", &Params::Config::fractionGeneratedNearest)
        .def_readwrite("fractionGeneratedFurthest", &Params::Config::fractionGeneratedFurthest)
        .def_readwrite("fractionGeneratedSweep", &Params::Config::fractionGeneratedSweep)
        .def_readwrite("fractionGeneratedRandomly", &Params::Config::fractionGeneratedRandomly)
        .def_readwrite("minSweepFillPercentage", &Params::Config::minSweepFillPercentage)
        .def_readwrite("maxToleratedCapacityViolation", &Params::Config::maxToleratedCapacityViolation)
        .def_readwrite("maxToleratedTimeWarp", &Params::Config::maxToleratedTimeWarp)
        .def_readwrite("initialTimeWarpPenalty", &Params::Config::initialTimeWarpPenalty)
        .def_readwrite("penaltyBooster", &Params::Config::penaltyBooster)
        .def_readwrite("minimumPopulationSize", &Params::Config::minimumPopulationSize)
        .def_readwrite("generationSize", &Params::Config::generationSize)
        .def_readwrite("nbElite", &Params::Config::nbElite)
        .def_readwrite("nbClose", &Params::Config::nbClose)
        .def_readwrite("targetFeasible", &Params::Config::targetFeasible)
        .def_readwrite("repairProbability", &Params::Config::repairProbability)
        .def_readwrite("growNbGranularAfterNonImprovementIterations", &Params::Config::growNbGranularAfterNonImprovementIterations)
        .def_readwrite("growNbGranularAfterIterations", &Params::Config::growNbGranularAfterIterations)
        .def_readwrite("growNbGranularSize", &Params::Config::growNbGranularSize)
        .def_readwrite("growPopulationAfterNonImprovementIterations", &Params::Config::growPopulationAfterNonImprovementIterations)
        .def_readwrite("growPopulationAfterIterations", &Params::Config::growPopulationAfterIterations)
        .def_readwrite("growPopulationSize", &Params::Config::growPopulationSize)
        .def_readwrite("diversityWeight", &Params::Config::diversityWeight)
        .def_readwrite("initialSolution", &Params::Config::initialSolution)
        .def_readwrite("nbVeh", &Params::Config::nbVeh)
        .def_readwrite("logPoolInterval", &Params::Config::logPoolInterval)
        .def_readwrite("isDimacsRun", &Params::Config::isDimacsRun)
        .def_readwrite("useDynamicParameters", &Params::Config::useDynamicParameters)
        .def_readwrite("pathSolution", &Params::Config::pathSolution)
        .def_readwrite("nbGranular", &Params::Config::nbGranular)
        .def_readwrite("intensificationProbabilityLS", &Params::Config::intensificationProbabilityLS)
        .def_readwrite("useSwapStarTW", &Params::Config::useSwapStarTW)
        .def_readwrite("skipSwapStarDist", &Params::Config::skipSwapStarDist)
        .def_readwrite("circleSectorOverlapToleranceDegrees", &Params::Config::circleSectorOverlapToleranceDegrees)
        .def_readwrite("minCircleSectorSizeDegrees", &Params::Config::minCircleSectorSizeDegrees)
        .def_readwrite("seed", &Params::Config::seed)
        .def_readwrite("pathInstance", &Params::Config::pathInstance)
        .def_readwrite("useSymmetricCorrelatedVertices", &Params::Config::useSymmetricCorrelatedVertices)
        .def_readwrite("doRepeatUntilTimeLimit", &Params::Config::doRepeatUntilTimeLimit);

    py::class_<Params>(m, "Params")
        .def(py::init<Params::Config &,
                      std::vector<std::pair<int, int>> const &,
                      std::vector<int> const &,
                      int,
                      std::vector<std::pair<int, int>> const &,
                      std::vector<int> const &,
                      std::vector<std::vector<int>> const &,
                      std::vector<int> const &>(),
             py::arg("config"),
             py::arg("coords"),
             py::arg("demands"),
             py::arg("vehicle_cap"),
             py::arg("time_windows"),
             py::arg("service_durations"),
             py::arg("duration_matrix"),
             py::arg("release_times"))
        .def("rng", [](Params &prm){ return prm.rng(); })
        .def("SetCorrelatedVertices", py::overload_cast<>(&Params::SetCorrelatedVertices))
        .def("SetCorrelatedVertices", py::overload_cast<std::vector<std::vector<int>>>(&Params::SetCorrelatedVertices))
        .def_readonly("correlatedVertices", &Params::correlatedVertices)
        .def_readonly("config", &Params::config)
		.def_readonly("nbVehicles", &Params::nbVehicles)
		.def_readonly("nbClients", &Params::nbClients)
        .def_readwrite("penaltyCapacity", &Params::penaltyCapacity)
        .def_readwrite("penaltyTimeWarp", &Params::penaltyTimeWarp)
        .def_readwrite("proximityWeightWaitTime", &Params::proximityWeightWaitTime)
        .def_readwrite("proximityWeightTimeWarp", &Params::proximityWeightTimeWarp);

    py::class_<Split>(m, "Split")
        .def(py::init<Params *>(), py::arg("params"))
		.def("generalSplit", &Split::generalSplit);

  py::class_<LocalSearch>(m, "LocalSearch")
      .def(py::init<Params *>(), py::arg("params"))
      .def("constructIndividualWithSeedOrder", &LocalSearch::constructIndividualWithSeedOrder)
      .def("constructIndividualBySweep", &LocalSearch::constructIndividualBySweep)
      .def("run", &LocalSearch::run);

  py::class_<Individual>(m, "Individual")
      .def(py::init<>())
      .def(py::init<Params*,bool>(), py::arg("params"), py::arg("initializeChromTAndShuffle"))
      .def(py::init<Params*,std::string>(), py::arg("params"), py::arg("solutionStr"))
      .def(py::init<Params*,std::vector<std::vector<int>> const &>(), py::arg("params"), py::arg("routes"))
      .def("__copy__",  [](const Individual &self) {
        return Individual(self);
      })
      .def("__deepcopy__", [](const Individual &self, py::dict) {
        return Individual(self);
      })
      .def_property_readonly("routes", [](Individual &indiv) {
        std::vector<std::vector<int>> routes;
        for (size_t i = 0; i < indiv.chromR.size(); i++){
            if (!indiv.chromR[i].empty())
            {
                routes.push_back(indiv.chromR[i]);
            }
        }
        return routes; 
      })
      .def_property_readonly("cost", [](Individual &indiv) { return indiv.myCostSol.penalizedCost; })
      .def("brokenPairsDistance", &Individual::brokenPairsDistance)
      .def_readonly("isFeasible", &Individual::isFeasible)
      .def_readonly("biasedFitness", &Individual::biasedFitness);
      
    py::class_<Population>(m, "Population")
        .def(py::init<Params *, Split *, LocalSearch *>(),
             py::arg("params"),
             py::arg("split"),
             py::arg("local_search"))
        .def("getBestFound", &Population::getBestFound, py::return_value_policy::reference)
        .def("generatePopulation", &Population::generatePopulation)
        .def("updateAllBiasedFitnesses", &Population::updateAllBiasedFitnesses)
        .def("reset", &Population::reset)
		.def("restart", &Population::restart)
		.def("printState", &Population::printState)
		.def("addIndividual", &Population::addIndividual)
        .def("getNonIdenticalParentsBinaryTournament", &Population::getNonIdenticalParentsBinaryTournament, py::return_value_policy::reference)
		.def("managePenalties", &Population::managePenalties)
        .def("size", &Population::size )
        .def("get", &Population::get, py::return_value_policy::reference )
        ;

    py::class_<Genetic>(m, "Genetic")
        .def(py::init<Params *, Split *, Population *, LocalSearch *>(),
             py::arg("params"),
             py::arg("split"),
             py::arg("population"),
             py::arg("local_search"))
        .def("run", &Genetic::run);

    py::class_<Crossover>(m, "Crossover")
        .def(py::init<Params*,Split*>())
		.def("OX", &Crossover::OX)
		.def("SREX", &Crossover::SREX)
		;
}
