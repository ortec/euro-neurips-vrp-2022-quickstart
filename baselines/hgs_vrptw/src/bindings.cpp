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
             py::arg("doRepeatUntilTimeLimit") = true);

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
             py::arg("release_times"));

    py::class_<Split>(m, "Split")
        .def(py::init<Params *>(), py::arg("params"));

  py::class_<LocalSearch>(m, "LocalSearch")
      .def(py::init<Params *>(), py::arg("params"));

  py::class_<Individual>(m, "Individual")
      .def("routes", [](Individual &indiv) { return indiv.chromR; })
      .def("cost", [](Individual &indiv) { return indiv.myCostSol.penalizedCost; });

    py::class_<Population>(m, "Population")
        .def(py::init<Params *, Split *, LocalSearch *>(),
            py::arg("params"),
             py::arg("split"),
           py::arg("local_search"))
        .def("getBestFound", &Population::getBestFound);

    py::class_<Genetic>(m, "Genetic")
        .def(py::init<Params *, Split *, Population *, LocalSearch *>(),
             py::arg("params"),
             py::arg("split"),
             py::arg("population"),
             py::arg("local_search"))
        .def("run", &Genetic::run,
             py::arg("maxIterNonProd"),
             py::arg("timeLimit"));
}
