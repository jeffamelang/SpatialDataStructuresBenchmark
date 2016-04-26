CC = g++

# include variables from other makefiles for each type of library
include *.sdbs.mk

CC_FLAGS = -Wall -Wextra -std=c++11 -O3

TARGETS = SpatialDataStructuresBenchmark

all: $(TARGETS)

SpatialDataStructuresBenchmark: SpatialDataStructuresBenchmark.cc PointScenarioGenerators.h $(VERSIONS) 
	$(CC) $< -o $@ $(CC_INCLUDE) $(LD_FLAGS) $(CC_FLAGS)

clean:
	rm -f $(TARGETS)

again: clean $(TARGETS)
