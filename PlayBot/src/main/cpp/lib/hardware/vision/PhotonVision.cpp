#include "lib/hardware/vision/PhotonVision.h"


PhotonVision::PhotonVision(std::string_view           cameraName,
            frc::Transform3d            robotToCamPose,
            frc::AprilTagFieldLayout    tagLayout,
            Eigen::Matrix<double, 3, 1> singleTagStdDevs,
            Eigen::Matrix<double, 3, 1> multiTagStdDevs,
            std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)> estConsumer) :
    cameraName{cameraName}, tagLayout{tagLayout}, photonEstimator
    { 
        tagLayout, 
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, 
        robotToCamPose
    },
    camera{cameraName},
    singleTagStdDevs{singleTagStdDevs},
    multiTagStdDevs{multiTagStdDevs},
    estConsumer{estConsumer}
{
    photonEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
    // Simulation setup, not really in a working state yet
    if (frc::RobotBase::IsSimulation())
    {
        visionSim = std::make_unique<photon::VisionSystemSim>("main");
        visionSim->AddAprilTags(tagLayout);
        cameraProp = std::make_unique<photon::SimCameraProperties>();
        cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
        cameraProp->SetCalibError(.35, .10);
        cameraProp->SetFPS(15_Hz);
        cameraProp->SetAvgLatency(50_ms);
        cameraProp->SetLatencyStdDev(15_ms);
        cameraSim = std::make_shared<photon::PhotonCameraSim>(&camera, *cameraProp.get());
        visionSim->AddCamera(cameraSim.get(), robotToCamPose);
        cameraSim->EnableDrawWireframe(true);
    }
}

photon::PhotonPipelineResult PhotonVision::GetLatestResult() 
{ 
    return m_latestResult; 
}

void PhotonVision::Periodic()
{
    // Run each new pipeline result through our pose estimator
    for (const auto &result : camera.GetAllUnreadResults())
    {
        // cache result and update pose estimator
        auto visionEst = photonEstimator.Update(result);
        m_latestResult = result;
        // In sim only, add our vision estimate to the sim debug field
        if (frc::RobotBase::IsSimulation())
        {
            if (visionEst)
            {
                GetSimDebugField().GetObject("VisionEstimation")->SetPose(visionEst->estimatedPose.ToPose2d());
            }
            else
            {
                GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
            }
        }
        if (visionEst)
        {
            estConsumer(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp,
                        GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d()));
        }
    }
}

Eigen::Matrix<double, 3, 1> PhotonVision::GetEstimationStdDevs(frc::Pose2d estimatedPose)
{
    Eigen::Matrix<double, 3, 1> estStdDevs = constants::vision::SingleTagStdDevs;
    auto                        targets    = GetLatestResult().GetTargets();
    int                         numTags    = 0;
    units::meter_t              avgDist    = 0_m;
    for (const auto &tgt : targets)
    {
        auto tagPose = photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
        if (tagPose)
        {
            numTags++;
            avgDist += tagPose->ToPose2d().Translation().Distance(estimatedPose.Translation());
        }
    }
    if (numTags == 0)
    {
        return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1)
    {
        estStdDevs = constants::vision::MultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m)
    {
        estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                        .finished();
    }
    else
    {
        estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
}

void PhotonVision::SimPeriodic(frc::Pose2d robotSimPose)
{
    visionSim->Update(robotSimPose);
}

void PhotonVision::ResetSimPose(frc::Pose2d pose)
{
    if (frc::RobotBase::IsSimulation())
    {
        visionSim->ResetRobotPose(pose);
    }
}

frc::Field2d& PhotonVision::GetSimDebugField() 
{ 
    return visionSim->GetDebugField(); 
}