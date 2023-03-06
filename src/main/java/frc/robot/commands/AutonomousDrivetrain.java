package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.function.Supplier;
import java.util.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.drivetrain;

public class AutonomousDrivetrain {
	private final drivetrain m_drive;
	private final Limelight m_limelight;
	private Supplier<Pose2d> poseSupplier;
	private final DifferentialDriveOdometry m_odometry;
	private final PIDController m_leftController = new PIDController(Constants.ArmkP, Constants.ArmkI, Constants.ArmkD);
	private final PIDController m_rightController = new PIDController(Constants.ArmkP, Constants.ArmkI, Constants.ArmkD);
	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts,
			Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);

	public AutonomousDrivetrain(drivetrain drive, Limelight limelight) {
		m_drive = drive;
		m_limelight = limelight;
		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), 0, 0);
		poseSupplier = limelight.getPose();
	}

	public void resetOdometry(Pose2d pose) {
		m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), 0, 0, pose);
		;
	}

	public void periodic() {
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_drive.getBackLeftEncoder().getDistance(),
				m_drive.getBackRightEncoder().getDistance());
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_drive.leftWheelSpeed(), m_drive.rightWheelSpeed());
	}

	public Command getAutonomousCommand(AprilTag target) {
		TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
				Constants.kMaxAccelerationMetersPerSecondSquared);
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				m_odometry.getPoseMeters(),
				Arrays.asList(
						new Translation2d(0, 0),
						new Translation2d(m_limelight.getDistance() - 5, 0)),
				new Pose2d(m_limelight.getDistance() - 5, 0, Rotation2d.fromDegrees(m_limelight.x)),
				config);

		RamseteCommand ramseteCommand = new RamseteCommand(trajectory, poseSupplier, null, m_feedforward, Constants.kDriveKinematics, null,
				m_leftController, m_rightController, null);
    // final Pos should be subtracted by a little bit
		return new ObstacleAvoidanceCommand(ramseteCommand, m_drive, m_limelight);
	}

	public Pose3d getPose() {
		return new Pose3d(poseSupplier.get());
	}

	private double getHeading() {
		return Math.IEEEremainder(m_drive.getGyro().getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
	}

  /*void AutoAlign(AprilTag target)
  {
    double theta_x = m_limelight.tx;
      while (Math.abs(gyro.getAngle() - theta_x) >= 5)
      {
        drivetrain.arcadeDrive(0, -0.65);
      }
      
    double theta_y = m_limelight.ty;
    distance = (limelight.height * Math.tan(theta_y)) * 0.95;
    drivetrain.arcadeDrive(distance, 0);
    
  } */
}
