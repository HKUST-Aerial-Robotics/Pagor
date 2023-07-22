//
// Created by qzj on 23-2-26.
//gtsam::Pose3

#ifndef SRC_POINT2POINTFACTOR_H
#define SRC_POINT2POINTFACTOR_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>
#include "CommonFunc.h"

namespace gtsam{
    class Point2pFactor : public NoiseModelFactor1<Pose3>  {

    private:
        typedef NoiseModelFactor1<Pose3> Base;

    public:
        Eigen::Vector3d msrc_, mtgt_; ///< src and tgt measurements

        Point2pFactor() {} ///< Default constructor for serialization
        Point2pFactor(Key X, Eigen::Vector3d src, Eigen::Vector3d tgt,
                      const SharedNoiseModel& model = nullptr) :
                Base(model, X) {
            msrc_ = src;
            mtgt_ = tgt;
        }
        ~Point2pFactor() override {}

        /// @return a deep copy of this factor
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Point2pFactor(*this))); }

        Vector evaluateError(const Pose3& X, boost::optional<Matrix&> H = boost::none) const {
            const gtsam::Rot3& R = X.rotation();
            gtsam::Vector3 mx = R * msrc_ + X.translation();
            gtsam::Vector3 error = mx - mtgt_;
            if (H) {
                *H = gtsam::Matrix(3, 6);
                (*H).block(0, 0, 3, 3) = - X.rotation().matrix() * skew(msrc_);
                (*H).block(0, 3, 3, 3) = X.rotation().matrix();
            }
            return error;
        }
    };
}


#endif //SRC_POINT2POINTFACTOR_H
