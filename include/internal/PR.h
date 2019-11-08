#pragma once
#ifndef LDSO_PR_H_
#define LDSO_PR_H_

#include "NumTypes.h"
#include "internal/CalibHessian.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/edge_pointxyz.h>



using namespace ldso::internal;

namespace ldso {

    struct Frame;

    // ---------------------------------------------------------------------------------------------------------
    // some g2o types will be used in pose graph
    // ---------------------------------------------------------------------------------------------------------

    class VertexPR : public g2o::BaseVertex<6, Sophus::SE3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPR() : g2o::BaseVertex<6, Sophus::SE3>() {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void setToOriginImpl() override {
            _estimate = Sophus::SE3();
        }

        virtual void oplusImpl(const double *update_) override {
            Vec6 update;
            update << update_[0], update_[1], update_[2], update_[3], update_[4], update_[5];
            _estimate = Sophus::SE3::exp(update) * _estimate;
        }

        inline Eigen::Matrix3d R() const {
            return _estimate.so3().matrix();
        }

        inline Eigen::Vector3d t() const {
            return _estimate.translation();
        }
    };

    class VertexSim3 : public g2o::BaseVertex<7, Sophus::Sim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexSim3() : g2o::BaseVertex<7, Sophus::Sim3>() {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void setToOriginImpl() override {
            _estimate = Sophus::Sim3();
        }

        virtual void oplusImpl(const double *update_) override {
            Vec7 update;
            update << update_[0], update_[1], update_[2], update_[3], update_[4], update_[5], update_[6];
            _estimate = Sophus::Sim3::exp(update) * _estimate;
        }
    };

    /**
     * point with inverse depth
     */
    class VertexPointInvDepth : public g2o::BaseVertex<1, double> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexPointInvDepth() : g2o::BaseVertex<1, double>() {};

        bool read(std::istream &is) { return true; }

        bool write(std::ostream &os) const { return true; }

        virtual void setToOriginImpl() {
            _estimate = 1.0;
        }

        virtual void oplusImpl(const double *update) {
            _estimate += update[0];
        }
    };


    // ---- Edges --------------------------------------------------------------------------------------------------

    /**
     * Edge of inverse depth prior for stereo-triangulated mappoints
     * Vertex: inverse depth map point
     *
     * Note: User should set the information matrix (inverse covariance) according to feature position uncertainty and baseline
     */
    class EdgeIDPPrior : public g2o::BaseUnaryEdge<1, double, VertexPointInvDepth> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeIDPPrior() : BaseUnaryEdge<1, double, VertexPointInvDepth>() {}

        virtual bool read(std::istream &is) override { return true; }

        virtual bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        // virtual void linearizeOplus() override;
    };

    /**
     * Odometry edge
     * err = T1.inv * T2
     */
    class EdgePR : public g2o::BaseBinaryEdge<6, Sophus::SE3, VertexPR, VertexPR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgePR() : g2o::BaseBinaryEdge<6, Sophus::SE3, VertexPR, VertexPR>() {}

        virtual bool read(std::istream &is) override { return true; }

        virtual bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override {
            Sophus::SE3 v1 = (static_cast<VertexPR *> (_vertices[0]))->estimate();
            Sophus::SE3 v2 = (static_cast<VertexPR *> (_vertices[1]))->estimate();
            _error = (_measurement.inverse() * v1 * v2.inverse()).log();
        };

        // jacobian implemented by numeric jacobian
    };

    /**
     * Monocular Sim3 edge
     */
    class EdgeSim3 : public g2o::BaseBinaryEdge<7, Sophus::Sim3, VertexSim3, VertexSim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeSim3() : g2o::BaseBinaryEdge<7, Sophus::Sim3, VertexSim3, VertexSim3>() {}

        virtual bool read(std::istream &is) override { return true; }

        virtual bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override {
            Sophus::Sim3 v1 = (static_cast<VertexSim3 *> (_vertices[0]))->estimate();
			Sophus::Sim3 v2 = (static_cast<VertexSim3 *> (_vertices[1]))->estimate();
            _error = (_measurement.inverse() * v1 * v2.inverse()).log();
        };

        virtual double initialEstimatePossible(
                const g2o::OptimizableGraph::VertexSet &, g2o::OptimizableGraph::Vertex *) { return 1.; }

        virtual void initialEstimate(
                const g2o::OptimizableGraph::VertexSet &from, g2o::OptimizableGraph::Vertex * /*to*/) {
            VertexSim3 *v1 = static_cast<VertexSim3 *>(_vertices[0]);
            VertexSim3 *v2 = static_cast<VertexSim3 *>(_vertices[1]);
            if (from.count(v1) > 0)
                v2->setEstimate(measurement().inverse() * v1->estimate());
            else
                v1->setEstimate(measurement() * v2->estimate());
        }
    };

    /**
    * Edge of reprojection error in one frame. Contain 3 vectices
    * Vertex 0: inverse depth map point
    * Veretx 1: Host KF PR
    * Vertex 2: Target KF PR
    **/
    class EdgePRIDP : public g2o::BaseMultiEdge<2, Vec2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // give the normalized x, y and camera intrinsics
        EdgePRIDP(double x, double y, std::shared_ptr<CalibHessian> &calib) : g2o::BaseMultiEdge<2, Vec2>() {
            resize(3);
            this->x = x;
            this->y = y;
            cam = calib;
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

        // virtual void linearizeOplus() override;

        bool isDepthValid() {
            return dynamic_cast<const VertexPointInvDepth *>( _vertices[0])->estimate() > 0;
        }

    protected:

        // [x,y] in normalized image plane in reference KF
        double x = 0, y = 0;
        bool linearized = false;

        std::shared_ptr<CalibHessian> cam;
    };

    class EdgeProjectPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectPoseOnly(const std::shared_ptr<Camera> cam, const Eigen::Vector3d &pw_)
                : g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPR>(), pw(pw_) {
            fx = cam->fx;
            fy = cam->fy;
            cx = cam->cx;
            cy = cam->cy;
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

    public:
        bool depthValid = true;
    private:
        double fx = 0, fy = 0, cx = 0, cy = 0;
        Eigen::Vector3d pw;    // world 3d position

    };

    class EdgeProjectPoseOnlySim3 : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexSim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectPoseOnlySim3(const std::shared_ptr<Camera> cam, const Eigen::Vector3d &pw_)
                : g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexSim3>(), pw(pw_) {
            fx = cam->fx;
            fy = cam->fy;
            cx = cam->cx;
            cy = cam->cy;
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

    public:
        bool depthValid = true;

    private:
        double fx = 0, fy = 0, cx = 0, cy = 0;
        Eigen::Vector3d pw;    // world 3d position

    };

    /**
     * error = measurement - Sim3*pw
     */
    class EdgePointSim3 : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexSim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgePointSim3(const Eigen::Vector3d &pw_)
                : g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexSim3>(), pw(pw_) {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override {

            const VertexSim3 *vSim3 = static_cast<VertexSim3 *> (vertex(0));
            Sophus::Sim3 Scw = vSim3->estimate();

            if (std::isnan(Scw.scale())) {
                LOG(INFO) << "Scw has nan: \n" << Scw.matrix() << std::endl;
                return;
            }

            _error = _measurement - Scw * pw;
        }

    private:
        Eigen::Vector3d pw;    // world 3d position

    };
}


#endif // LDSO_PR_H_
