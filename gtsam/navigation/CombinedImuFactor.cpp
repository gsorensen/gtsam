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

//------------------------------------------------------------------------------
// Inner class PreintegrationCombinedParams
//------------------------------------------------------------------------------
void PreintegrationCombinedParams::print(const string& s) const {
  PreintegrationParams::print(s);
  cout << "biasAccCovariance:\n[\n" << biasAccCovariance << "\n]" << endl;
  cout << "biasOmegaCovariance:\n[\n" << biasOmegaCovariance << "\n]" << endl;
}

//------------------------------------------------------------------------------
bool PreintegrationCombinedParams::equals(
    const PreintegratedRotationParams& other, double tol) const {
  auto e = dynamic_cast<const PreintegrationCombinedParams*>(&other);
  return e != nullptr && PreintegrationParams::equals(other, tol) &&
         equal_with_abs_tol(biasAccCovariance, e->biasAccCovariance, tol) &&
         equal_with_abs_tol(biasOmegaCovariance, e->biasOmegaCovariance, tol);
}

}  // namespace gtsam
