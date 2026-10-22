#ifndef EIGENQUATERNIONPARAMETERIZATION_H
#define EIGENQUATERNIONPARAMETERIZATION_H

#include <ceres/manifold.h>

namespace camodocal
{

// Ceres 2.2 removed LocalParameterization. EigenQuaternionManifold matches the
// old camodocal map: Eigen order (x, y, z, w) and a left-multiplied exp map.
using EigenQuaternionParameterization = ceres::EigenQuaternionManifold;

}

#endif
