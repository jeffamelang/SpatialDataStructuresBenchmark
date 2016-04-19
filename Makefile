CC = g++

STLIB_INCLUDE_PATH = /clinic/2015/sandia15/source/stlib
PCL_INCLUDE_PATH = /clinic/2015/sandia15/pcl/usr/local/include/pcl-1.8/
PCL_LIB_PATH = /clinic/2015/sandia15/pcl/usr/local/lib
KDTREE2_INCLUDE_PATH = /clinic/2015/sandia15/something
KDTREE2_LIB_PATH = /clinic/2015/sandia15/something

CC_INCLUDE = -I$(PCL_INCLUDE_PATH) -I$(STLIB_INCLUDE_PATH) -I$(KDTREE2_INCLUDE_PATH)
CC_FLAGS = -Wall -Wextra -std=c++11 -O3
LD_FLAGS = -L$(PCL_LIB_PATH) -lpcl_kdtree -lpcl_octree -L$(KDTREE2_LIB_PATH) -lkdtree2

TARGETS = SpatialDataStructuresBenchmark

all: $(TARGETS)

SpatialDataStructuresBenchmark: SpatialDataStructuresBenchmark.cc PointScenarioGenerators.h Versions_stlib.h Versions_pcl.h Versions_kdtree2.h
	$(CC) $< -o $@ $(CC_INCLUDE) $(LD_FLAGS) $(CC_FLAGS)

clean:
	rm -f $(TARGETS)

again: clean $(TARGETS)
