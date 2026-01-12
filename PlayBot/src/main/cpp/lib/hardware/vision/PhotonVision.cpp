#include "lib/hardware/vision/PhotonVision.h"

PhotonVision::PhotonVision(std::string_view            cameraName,
                           frc::Transform3d            robotToCameraPose,
                           frc::AprilTagFieldLayout    tagLayout,
                           Eigen::Matrix<double, 3, 1> singleTagStdDevs,
                           Eigen::Matrix<double, 3, 1> multiTagStdDevs,
                           std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)> estConsumer) :
    m_cameraName{cameraName}, 
    m_tagLayout{tagLayout},
    m_photonEstimator
    { 
        tagLayout, 
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, 
        robotToCameraPose
    },
    m_camera{cameraName},
    m_singleTagStdDevs{singleTagStdDevs},
    m_multiTagStdDevs{multiTagStdDevs},
    m_estConsumer{estConsumer}
{
    m_photonEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

    // Simulation setup, not really in a working state yet
    if (frc::RobotBase::IsSimulation())
    {
        m_visionSim = std::make_unique<photon::VisionSystemSim>("main");
        m_visionSim->AddAprilTags(tagLayout);
        m_cameraProp = std::make_unique<photon::SimCameraProperties>();
        m_cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
        m_cameraProp->SetCalibError(.35, .10);
        m_cameraProp->SetFPS(15_Hz);
        m_cameraProp->SetAvgLatency(50_ms);
        m_cameraProp->SetLatencyStdDev(15_ms);
        m_cameraSim = std::make_shared<photon::PhotonCameraSim>(&m_camera, *m_cameraProp.get());
        m_visionSim->AddCamera(m_cameraSim.get(), robotToCameraPose);
        m_cameraSim->EnableDrawWireframe(true);
    }
}

photon::PhotonPipelineResult PhotonVision::GetLatestResult() 
{ 
    return m_latestResult; 
}

void PhotonVision::Periodic()
{
    // Run each new pipeline result through our pose estimator
    for (const auto &result : m_camera.GetAllUnreadResults())
    {
        // Remember the latest result
        m_latestResult = result;

        // cache result and update pose estimator
        auto visionEstimator = m_photonEstimator.Update(m_latestResult);

        // In sim only, add our vision estimate to the sim debug field
        if (frc::RobotBase::IsSimulation())
        {
            if (visionEstimator)
            {
                GetSimDebugField().GetObject("VisionEstimation")->SetPose(visionEstimator->estimatedPose.ToPose2d());
            }
            else
            {
                GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
            }
        }
        
        if (visionEstimator)
        {
            m_estConsumer(visionEstimator->estimatedPose.ToPose2d(), visionEstimator->timestamp,
                          GetEstimationStdDevs(visionEstimator->estimatedPose.ToPose2d()));
        }
    }
}

Eigen::Matrix<double, 3, 1> PhotonVision::GetEstimationStdDevs(frc::Pose2d estimatedPose)
{
    Eigen::Matrix<double, 3, 1> estimatedStdDevs = constants::vision::SingleTagStdDevs;
    auto                        targets          = GetLatestResult().GetTargets();
    int                         numberOfTags     = 0;
    units::meter_t              averageDistance  = 0_m;

    for (const auto &target : targets)
    {
        auto tagPose = m_photonEstimator.GetFieldLayout().GetTagPose(target.GetFiducialId());

        if (tagPose)
        {
            numberOfTags++;
            averageDistance += tagPose->ToPose2d().Translation().Distance(estimatedPose.Translation());
        }
    }

    if (numberOfTags == 0)
    {
        return estimatedStdDevs;
    }

    averageDistance /= numberOfTags;

    if (numberOfTags > 1)
    {
        estimatedStdDevs = constants::vision::MultiTagStdDevs;
    }
    
    if (numberOfTags == 1 && averageDistance > 4_m)
    {
        estimatedStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                        .finished();
    }
    else
    {
        estimatedStdDevs = estimatedStdDevs * (1 + (averageDistance.value() * averageDistance.value() / 30));
    }
    return estimatedStdDevs;
}

void PhotonVision::SimPeriodic(frc::Pose2d robotSimPose)
{
    m_visionSim->Update(robotSimPose);
}

void PhotonVision::ResetSimPose(frc::Pose2d pose)
{
    if (frc::RobotBase::IsSimulation())
    {
        m_visionSim->ResetRobotPose(pose);
    }
}

frc::Field2d& PhotonVision::GetSimDebugField() 
{ 
    return m_visionSim->GetDebugField(); 
}
