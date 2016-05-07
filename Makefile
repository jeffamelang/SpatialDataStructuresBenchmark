CC = g++

CC_FLAGS = -Wall -Wextra -std=c++11 -O3

# include variables from other makefiles for each type of library
include *.sdbs.mk

TARGETS = SpatialDataStructuresBenchmark

all: $(TARGETS)

SpatialDataStructuresBenchmark: SpatialDataStructuresBenchmark.cc PointScenarioGenerators.h $(VERSIONS) 
	$(CC) $< -o $@ $(CC_INCLUDE) $(LD_FLAGS) $(CC_FLAGS)

clean:
	rm -f $(TARGETS) *.o

again: clean $(TARGETS)
