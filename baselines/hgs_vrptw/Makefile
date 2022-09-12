all : genvrp

CCC = g++
# CCC = clang++-10
CCFLAGS = -O3 -Wall -std=c++2a

# CCFLAGS = -Wall -std=c++2a -g
TARGETDIR=.
SRCDIR=src
INCLUDEDIR=include

OBJS2 = \
        $(TARGETDIR)/Genetic.o \
        $(TARGETDIR)/Individual.o \
        $(TARGETDIR)/LocalSearch.o \
        $(TARGETDIR)/main.o \
        $(TARGETDIR)/Params.o \
        $(TARGETDIR)/Population.o \
        $(TARGETDIR)/Split.o

$(TARGETDIR)/genvrp: $(OBJS2)
	$(CCC) $(CCFLAGS) -o $(TARGETDIR)/genvrp $(OBJS2)
	
$(TARGETDIR)/Genetic.o: $(INCLUDEDIR)/Genetic.h $(SRCDIR)/Genetic.cpp
	$(CCC) $(CCFLAGS) -c $(SRCDIR)/Genetic.cpp -o $(TARGETDIR)/Genetic.o -I$(INCLUDEDIR)/

$(TARGETDIR)/Individual.o: $(INCLUDEDIR)/Individual.h $(SRCDIR)/Individual.cpp
	$(CCC) $(CCFLAGS) -c $(SRCDIR)/Individual.cpp -o $(TARGETDIR)/Individual.o -I$(INCLUDEDIR)/

$(TARGETDIR)/LocalSearch.o: $(INCLUDEDIR)/LocalSearch.h $(SRCDIR)/LocalSearch.cpp
	$(CCC) $(CCFLAGS) -c $(SRCDIR)/LocalSearch.cpp -o $(TARGETDIR)/LocalSearch.o -I$(INCLUDEDIR)/
	
$(TARGETDIR)/main.o: $(SRCDIR)/main.cpp
	$(CCC) $(CCFLAGS) -c $(SRCDIR)/main.cpp -o $(TARGETDIR)/main.o -I$(INCLUDEDIR)/

$(TARGETDIR)/Params.o: $(INCLUDEDIR)/Params.h $(SRCDIR)/Params.cpp
	$(CCC) $(CCFLAGS) -c $(SRCDIR)/Params.cpp -o $(TARGETDIR)/Params.o -I$(INCLUDEDIR)/

$(TARGETDIR)/Population.o: $(INCLUDEDIR)/Population.h $(SRCDIR)/Population.cpp
	$(CCC) $(CCFLAGS) -c $(SRCDIR)/Population.cpp -o $(TARGETDIR)/Population.o -I$(INCLUDEDIR)/

$(TARGETDIR)/Split.o: $(INCLUDEDIR)/Split.h $(SRCDIR)/Split.cpp
	$(CCC) $(CCFLAGS) -c $(SRCDIR)/Split.cpp -o $(TARGETDIR)/Split.o -I$(INCLUDEDIR)/

test: genvrp
	./genvrp ../../instances/ORTEC-VRPTW-ASYM-0bdff870-d1-n458-k35.txt test.sol -seed 1 

clean:
	$(RM) \
    $(TARGETDIR)/main.o \
    $(TARGETDIR)/Genetic.o \
    $(TARGETDIR)/Individual.o \
    $(TARGETDIR)/LocalSearch.o \
    $(TARGETDIR)/Params.o \
    $(TARGETDIR)/Population.o \
    $(TARGETDIR)/Split.o