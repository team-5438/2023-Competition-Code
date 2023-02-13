package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrain;

public class ObstacleAvoidanceCommand extends CommandBase {

	private final drivetrain m_drivetrain;
	private final Limelight m_limelight;

	public ObstacleAvoidanceCommand(RamseteCommand ramseteCommand, drivetrain drivetrain, Limelight limelight) {
		m_drivetrain = drivetrain;
		m_limelight = limelight;

		addRequirements(m_drivetrain);
	}

	@Override
	public void execute() {
		// Get the current pose of the robot
		Pose2d currentPose = (Pose2d) m_drivetrain.getPose();

		// Get the distance to the nearest obstacle
		double distanceToObstacle = m_limelight.getDistance();

		// Calculate the desired speed and direction to avoid the obstacle
		double speed = calculateAvoidanceSpeed(distanceToObstacle);
		Rotation2d rotation = calculateAvoidanceDirection(currentPose, distanceToObstacle);
		double rotationNum = rotation.getDegrees();

		// Drive the robot in the desired direction at the desired speed
		m_drivetrain.arcadeDrive(speed, rotationNum);
	}

	private double calculateAvoidanceSpeed(double distanceToObstacle) {

		double distanceFromObstacle = m_limelight.getDistance();
		if (distanceFromObstacle >= 5) {
			return 0.3;
		} else if (distanceFromObstacle < 5 && distanceFromObstacle >= 2) {
			return 0.5;
		} else {
			return 0.8;
			// TODO: Tune these values later
		}
	}

	private Rotation2d calculateAvoidanceDirection(Pose2d currentPose, double distanceToObstacle) {
		// TODO: Determine the best direction to drive the robot to avoid the obstacle
		// Use the Limelight's distance to the obstacle and the current pose of the
		// robot to determine the best direction
		m_limelight.getDistance();
		return null;
	}
}
