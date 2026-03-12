/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactor.cpp
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 *  @author Varun Agrawal
 **/

#include <gtsam/navigation/CombinedImuFactor.h>
#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/export.hpp>
#endif

namespace gtsam {

using namespace std;

// Explicit instantiations for ConstantBias
template struct PreintegrationCombinedParamsT<imuBias::ConstantBias>;

// Explicit instantiations for GaussMarkovBias
template struct PreintegrationCombinedParamsT<imuBias::GaussMarkovBias>;

}  // namespace gtsam
