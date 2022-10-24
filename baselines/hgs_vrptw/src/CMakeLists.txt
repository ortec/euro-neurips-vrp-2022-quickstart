cmake_minimum_required(VERSION 3.2)

set(geneticSources
        Genetic.cpp)

set(individualSources
        Individual.cpp)

set(localSearchSources
        LocalSearch.cpp)

set(paramsSources
        Params.cpp)

set(populationSources
        Population.cpp)

set(splitSources
        Split.cpp)

set(crossoverSources
        Crossover.cpp)

# (Internal) library
add_library(hgs
        ${geneticSources}
        ${individualSources}
        ${localSearchSources}
        ${paramsSources}
        ${populationSources}
        ${splitSources}
        ${crossoverSources})

# Executable
add_executable(genvrp main.cpp)
target_link_libraries(genvrp PRIVATE hgs)

# Python bindings
pybind11_add_module(hgspy bindings.cpp)
target_link_libraries(hgspy PRIVATE hgs)
