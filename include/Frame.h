#pragma once
#ifndef LDSO_FRAME_H_
#define LDSO_FRAME_H_

#include <vector>
#include <memory>
#include <set>
#include <mutex>


#include "NumTypes.h"
#include "AffLight.h"

namespace ldso {

    // forward declare
    struct Feature;
    namespace internal {
        class FrameHessian;
    }
    struct Point;

    /**
     * Frame is the basic element holding the pose and image data.
     * Here is only the minimal required data, the inner structures are stored in FrameHessian
     */
    struct Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Frame();

        /**
         * Constructro with timestamp
         * @param timestamp
         */
        Frame(double timestamp);

        /**
         * this internal structure should be created if you want to use them
         */
        void CreateFH(std::shared_ptr<Frame> frame);

        /**
         * release the inner structures
         * NOTE: call release when you want to actually delete the internal data, otherwise it will stay in memory forever
         * (because the FrameHessian is also holding its shared pointer)
         */
        void ReleaseFH();

        /**
         * release the internal structure in features and map points, call this if you no longer want them
         */
        void ReleaseFeatures();

        /**
         * Release all the internal structures in FrameHessian, PointHessian and immature points
         * called when this frame is marginalized
         */
        inline void ReleaseAll() {
            ReleaseFH();
            ReleaseFeatures();
        }

        /**
         * Set the feature grid
         */
        void SetFeatureGrid();

        /**
         * get the feature indecies around a given point
         * @param x
         * @param y
         * @param radius
         * @return
         */
		std::vector<size_t> GetFeatureInGrid(const float &x, const float &y, const float &radius);

        /**
         * compute bow vectors
         * @param voc vocabulary pointer
         */
        void ComputeBoW(std::shared_ptr<ORBVocabulary> voc);

        // get keyframes in window
		std::set<std::shared_ptr<Frame>> GetConnectedKeyFrames();

        // get all associated points
		std::vector<std::shared_ptr<Point>> GetPoints();

        // save & load
        void save(std::ofstream &fout);    // this will save all the map points
        void load(std::ifstream &fin, std::shared_ptr<Frame> &thisFrame, std::vector<std::shared_ptr<Frame>> &allKF);

        // get and write pose
        SE3 getPose() {
			std::unique_lock<std::mutex> lck(poseMutex);
            return Tcw;
        }

        void setPose(const SE3 &Tcw) {
			std::unique_lock<std::mutex> lck(poseMutex);
            this->Tcw = Tcw;
        }

        // get and write the optimized pose by loop closing
        Sim3 getPoseOpti() {
			std::unique_lock<std::mutex> lck(poseMutex);
            return TcwOpti;
        }

        void setPoseOpti(const Sim3 &Scw) {
			std::unique_lock<std::mutex> lck(poseMutex);
            TcwOpti = Scw;
        }

        // =========================================================================================================
        // data
        unsigned long id = 0;        // id of this frame
        static unsigned long nextId;  // next id
        unsigned long kfId = 0;      // keyframe id of this frame

    private:
        // poses
        // access them by getPose and getPoseOpti function
		std::mutex poseMutex;            // need to lock this pose since we have multiple threads reading and writing them
        SE3 Tcw;           // pose from world to camera, estimated by DSO (nobody wants to touch DSO's backend except Jakob)
        Sim3 TcwOpti;     // pose from world to camera optimized by global pose graph (with scale)

    public:
        bool poseValid = true;     // if pose is valid (false when initializing)
        double timeStamp = 0;      // time stamp
        AffLight aff_g2l;           // aff light transform from global to local
		std::vector<std::shared_ptr<Feature>> features;  // Features contained
		std::vector<std::vector<std::size_t>> grid;      // feature grid, to fast access features in a given area
        const int gridSize = 20;                // grid size

        // pose relative to keyframes in the window, stored as T_cur_ref
        // this will be changed by full system and loop closing, so we need a mutex
        std::mutex mutexPoseRel;

        /**
         * Relative pose constraint between key-frames
         */
        struct RELPOSE {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            RELPOSE(Sim3 T = Sim3(), const Mat77 &H = Mat77::Identity(), bool bIsLoop = false) :
                    Tcr(T), isLoop(bIsLoop), info(H) {}

            Sim3 Tcr;    // T_current_reference
            Mat77 info = Mat77::Identity();  // information matrix, inverse of covariance, default is identity
            bool isLoop = false;
        };

        // relative poses within the active window
		std::map<std::shared_ptr<Frame>, RELPOSE, std::less<std::shared_ptr<Frame>>, Eigen::aligned_allocator<std::pair<const std::shared_ptr<Frame>, RELPOSE>>> poseRel;

        // Bag of Words Vector structures.
        DBoW3::BowVector bowVec;       // BoW Vector
        DBoW3::FeatureVector featVec;  // Feature Vector
		std::vector<size_t> bowIdx;         // index of the bow-ized corners

		std::shared_ptr<internal::FrameHessian> frameHessian = nullptr;  // internal data

        // ===== debug stuffs ======= //
        cv::Mat imgDisplay;    // image to display, only for debugging, remain an empty image if setting_show_loopclosing is false
    };

    /**
     * Compare frame ID, used to get a sorted map or set of frames
     */
    class CmpFrameID {
    public:
        inline bool operator()(const std::shared_ptr<Frame> &f1, const std::shared_ptr<Frame> &f2) const{
            return f1->id < f2->id;
        }
    };

    /**
     * Compare frame by Keyframe ID, used to get a sorted keyframe map or set.
     */
    class CmpFrameKFID {
    public:
        inline bool operator()(const std::shared_ptr<Frame> &f1, const std::shared_ptr<Frame> &f2)const {
            return f1->kfId < f2->kfId;
        }
    };
}

#endif// LDSO_FRAME_H_
