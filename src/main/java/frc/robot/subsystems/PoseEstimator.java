/*package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase
{
  private DifferentialDrivePoseEstimator poseEstimator;
  private PhotonPoseEstimator photonPoseEstimator;
  
  drivetrain drive;
  private Field2d field;
  private Vector<N3> stateEstimate = VecBuilder.fill(0.05, 0.05, 0.01);
  private Vector<N3> visionEstimate = VecBuilder.fill(0.5, 0.5, 1);
  double pipelineTimestamp = 0;

  public PoseEstimator(drivetrain m_drive)
  {
    drive = m_drive;
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    poseEstimator = new DifferentialDrivePoseEstimator(
      Constants.kDriveKinematics, 
      drive.getNavxAngle(), 
      drive.getWheelPositions(),
      new Pose2d(0, 0, new Rotation2d(Math.PI)),
      stateEstimate,
      visionEstimate
    );
  }

  @Override
  public void periodic()
  {
    poseEstimator.update(drive.getNavxAngle(), drive.getWheelPositions());

    if (photonPoseEstimator != null)
    {
      photonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
        var estimatedPose = estimatedRobotPose.estimatedPose;

      
  }
  
  public Pose2d getPose()
  {
    return poseEstimator.getEstimatedPosition();
  }
  
  public void setPose(Pose2d newPose)
  {
    poseEstimator.resetPosition(drive.getNavxAngle(), drive.getWheelPositions(), new Pose2d());
  }
  
  public void resetPosition()
  {
    setPose(new Pose2d());
  }
}
*/