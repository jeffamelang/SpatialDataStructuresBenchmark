PCL_INCLUDE_PATH = /research/jeff/pcl/usr/local/include/pcl-1.8/
PCL_LIB_PATH = /research/jeff/pcl/usr/local/lib

CC_INCLUDE += -isystem $(PCL_INCLUDE_PATH)
LD_FLAGS += -L$(PCL_LIB_PATH) -lpcl_kdtree -lpcl_octree 
VERSIONS += Versions_pcl.h
