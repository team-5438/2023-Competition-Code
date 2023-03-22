package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase
{
  private DifferentialDrivePoseEstimator poseEstimator;
  private PhotonPoseEstimator photonPoseEstimator;
  private Vision vision;
  
  drivetrain drive;
  private Field2d field;
  private Vector<N3> stateEstimate = VecBuilder.fill(0.05, 0.05, 0.01);
  private Vector<N3> visionEstimate = VecBuilder.fill(0.5, 0.5, 1);
  double pipelineTimestamp = 0;

  public PoseEstimator(drivetrain m_drive)
  {
    drive = m_drive;
    field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    poseEstimator = new DifferentialDrivePoseEstimator(Constants.kDriveKinematics);


  }

  public Pose2d getPose()
  {
    return poseEstimator.getEstimatedPosition();
  }
  
  public void setPose(Pose2d newPose)
  {
    poseEstimator.resetPosition(drive.getNavxAngle(), drive.getWheelPositions(), newPose2d());
  }
  
  public void resetPosition()
  {
    setPose(new Pose2d());
  }
}