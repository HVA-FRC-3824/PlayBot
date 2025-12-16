/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <functional>
#include <limits>
#include <memory>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Constants.h"

class PhotonVision
{
    public:

        PhotonVision(std::string_view            cameraName,
                     frc::Transform3d            robotToCameraPose,
                     frc::AprilTagFieldLayout    tagLayout,
                     Eigen::Matrix<double, 3, 1> singleTagStdDevs,
                     Eigen::Matrix<double, 3, 1> multiTagStdDevs,
                     std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)> estConsumer);
        
        photon::PhotonPipelineResult GetLatestResult() ;

        void                         Periodic();

        Eigen::Matrix<double, 3, 1>  GetEstimationStdDevs(frc::Pose2d estimatedPose);

        void                         SimPeriodic(frc::Pose2d robotSimPose);

        void                         ResetSimPose(frc::Pose2d pose);

        frc::Field2d&                GetSimDebugField();

    private:

        std::string_view                             m_cameraName;
        frc::Transform3d                             m_robotToCameraPose;
        frc::AprilTagFieldLayout                     m_tagLayout;

        photon::PhotonPoseEstimator                  m_photonEstimator;

        photon::PhotonCamera                         m_camera;

        Eigen::Matrix<double, 3, 1>                  m_singleTagStdDevs;
        Eigen::Matrix<double, 3, 1>                  m_multiTagStdDevs;

        std::unique_ptr<photon::VisionSystemSim>     m_visionSim;
        std::unique_ptr<photon::SimCameraProperties> m_cameraProp;
        std::shared_ptr<photon::PhotonCameraSim>     m_cameraSim;

        // The most recent result, cached for calculating std devs
        photon::PhotonPipelineResult                 m_latestResult;
        
        std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)> m_estConsumer;
};
