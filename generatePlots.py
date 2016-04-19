import math
import os
import sys
import numpy
import scipy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.cm as cm
import csv
from mpl_toolkits.mplot3d import Axes3D
from numpy import log10

pointGeneratorName = 'uniformRandomWithAverageNumberOfNeighbors'
prefix = 'data/SpatialDataStructuresBenchmark_' + pointGeneratorName + '_'
suffix = '_knuth'
outputPrefix = 'figures/SpatialDataStructuresBenchmark_' + pointGeneratorName + '_'

stlib_cellArray = numpy.loadtxt(open(prefix + 'stlib_cellArray_results' + suffix + '.csv','rb'),delimiter=',',skiprows=0)
stlib_octTree = numpy.loadtxt(open(prefix + 'stlib_octTree_results' + suffix + '.csv','rb'),delimiter=',',skiprows=0)
stlib_kdTree = numpy.loadtxt(open(prefix + 'stlib_kdTree_results' + suffix + '.csv','rb'),delimiter=',',skiprows=0)
pcl_kdTree = numpy.loadtxt(open(prefix + 'pcl_kdTree_results' + suffix + '.csv','rb'),delimiter=',',skiprows=0)
pcl_ocTree = numpy.loadtxt(open(prefix + 'pcl_ocTree_results' + suffix + '.csv','rb'),delimiter=',',skiprows=0)
#kdtree2 = numpy.loadtxt(open(prefix + 'kdtree2_results' + suffix + '.csv','rb'),delimiter=',',skiprows=0)

for column in range(1, 3):
  if column == 1:
    flavorName = 'initialization'
  else:
    flavorName = 'querying'
  plt.figure(figsize=(9, 6))
  ax = plt.subplot(111)
  plt.yscale('log')
  plt.xscale('log')
  legendNames = []
  plt.plot(stlib_cellArray[:,0], stlib_cellArray[:,column] / stlib_octTree[:,column], color='k', hold='on', linewidth=2)
  legendNames.append('stlib_octTree')
  plt.plot(stlib_cellArray[:,0], stlib_cellArray[:,column] / stlib_kdTree[:,column], color='b', hold='on', linewidth=2)
  legendNames.append('stlib_kdTree')
  plt.plot(stlib_cellArray[:,0], stlib_cellArray[:,column] / pcl_kdTree[:,column], color='b', linestyle='dashed', hold='on', linewidth=2)
  legendNames.append('pcl_kdTree')
  plt.plot(stlib_cellArray[:,0], stlib_cellArray[:,column] / pcl_ocTree[:,column], color='k', linestyle='dashed', hold='on', linewidth=2)
  legendNames.append('pcl_ocTree')
  #plt.plot(stlib_cellArray[:,0], stlib_cellArray[:,column] / kdtree2[:,column], color='b', linestyle='dashdot', hold='on', linewidth=2)
  #legendNames.append('kdtree2')
  plt.title('speedup of ' + flavorName, fontsize=16)
  plt.xlabel('number of points [-]', fontsize=16)
  plt.ylabel('speedup [-]', fontsize=16)
  box = ax.get_position()
  ax.set_position([box.x0, box.y0, box.width * 0.70, box.height])
  ax.legend(legendNames, loc='upper left', bbox_to_anchor=(1.00, 0.80))
  filename = outputPrefix + flavorName + suffix + '.pdf'
  plt.savefig(filename)
  print 'saved file to %s' % filename
