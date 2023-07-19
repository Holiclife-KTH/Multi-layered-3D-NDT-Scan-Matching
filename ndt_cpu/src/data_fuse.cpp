#include "ndt_cpu/VoxelGrid.h"
#include "ndt_cpu/debug.h"
#include <math.h>
#include <limits>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>

#include "ndt_cpu/SymmetricEigenSolver.h"

template <typename PointSourceType>
float VoxelGrid<PointSourceType>::get_mean()
{
  return mean;
}

template <typename PointSourceType>
Eigen::Matrix3d VoxelGrid<PointSourceType>::getInverseCovariance(int voxel_id) const
{
  return (*icovariance_)[voxel_id];
}

template <typename PointSourceType>
void VoxelGrid<PointSourceType>::fuse()
{