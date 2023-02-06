package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.drivetrain;
import frc.robot.Constants;


public class AutonomousDrivetrain {
    private final drivetrain m_drive;
    private final Limelight m_limelight;
    private final Pose2d pose;
    private final DifferentialDriveOdometry m_odometry;
    private final PIDController m_leftController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    private final PIDController m_rightController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
    private final SimpleMotorFeedforward m_feedforward =
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);
  
   
    public AutonomousDrivetrain(drivetrain drive, Limelight limelight) {
      m_drive = drive;
      m_limelight = limelight;
      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), 0, 0);
    }
  
    public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), 0, 0, pose);;
    }
  
    public void periodic() {
      m_odometry.update(Rotation2d.fromDegrees(getHeading()), 		m_drive.getBackLeftEncoder().getDistance(),
          m_drive.getBackRightEncoder().getDistance());
    }
  
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds( m_drive.leftWheelSpeed(), m_drive.rightWheelSpeed());
  } 
  public Command getAutonomousCommand(AprilTag target) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        m_odometry.getPoseMeters(),
        Arrays.asList(
            new Translation2d(0, 0),
            new Translation2d(m_limelight.getDistance() - 5, 0)
        ),
        new Pose2d(m_limelight.getDistance() - 5, 0, Rotation2d.fromDegrees(m_limelight.x)),
        config
    );
  
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, pose, null, m_feedforward, null, null, m_leftController, m_rightController, null, null)
    new RamseteCommand(
      trajectory,
      m_drive::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      m_feedforward,
      Constants.kDriveKinematics,
      m_drive::getWheelSpeeds,
      m_leftController,
      m_rightController,
      m_drive::tankDriveVolts,
      m_drive
  );
    return new ObstacleAvoidanceCommand(ramseteCommand, m_drive, m_limelight);
  }
  
  private double getHeading() {
    return Math.IEEEremainder(m_drive.getGyro().getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }
  }
