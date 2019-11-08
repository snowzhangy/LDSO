add_library(
        dso
        STATIC
)
add_library(dso::DSO ALIAS dso)
target_sources(
        dso
        PRIVATE
        Feature.cc
        Frame.cc
        Point.cc
        Setting.cc
        Camera.cc
        Map.cc

        internal/PointHessian.cc
        internal/FrameHessian.cc
        internal/GlobalCalib.cc
        internal/FrameFramePrecalc.cc
        internal/Residuals.cc
        internal/ImmaturePoint.cc
        internal/PR.cc

        internal/OptimizationBackend/AccumulatedSCHessian.cc
        internal/OptimizationBackend/AccumulatedTopHessian.cc
        internal/OptimizationBackend/EnergyFunctional.cc

        frontend/CoarseTracker.cc
        frontend/CoarseInitializer.cc
        frontend/FullSystem.cc
        frontend/DSOViewer.cc
        frontend/FeatureDetector.cc
        frontend/FeatureMatcher.cc
        frontend/LoopClosing.cc
        frontend/PixelSelector2.cc
        frontend/Undistort.cc
        frontend/ImageRW_OpenCV.cc
)

target_include_directories(
        dso
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
        ${OpenCV_INCLUDE_DIRS}
		${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(
        dso
        PUBLIC
        Sophus::Sophus
        pangolin
        ${OpenCV_LIBS}
        DBow3::DBow3
        Eigen3::Eigen
        SuiteSparse::cholmod
		fmt::fmt
        glog::glog
		dso::options
        dso::SIMD
		g2o::core
		g2o::types_slam3d
		g2o::solver_csparse
		g2o::solver_cholmod
		g2o::solver_eigen
)

target_compile_features(dso PUBLIC cxx_std_17)

target_compile_definitions(
        dso
        PUBLIC
        ENABLE_SSE
        $<$<PLATFORM_ID:Windows>:M_PI=3.14159265359>
        $<$<PLATFORM_ID:Windows>:EIGEN_NO_DEPRECATED_WARNING>
        $<$<PLATFORM_ID:Windows>:__SSE3__>
)

if (NOT WIN32)
    target_compile_options(
            dso
            PUBLIC
            -fPIC
    )
endif ()
