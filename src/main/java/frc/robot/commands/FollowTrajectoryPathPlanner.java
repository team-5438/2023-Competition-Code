package frc.robot.commands;

import frc.robot.subsystems.*;
import com.pathplanner.lib.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class FollowTrajectoryPathPlanner extends CommandBase {
  private drivetrain drive;
  private PoseEstimator poseEstimator;
  private String pathName;
  private PathConstraints constraints;

  private CommandBase controllerCommand = Commands.none();

  public FollowTrajectoryPathPlanner(drivetrain m_drive, PoseEstimator pe, String name, PathConstraints consts) {
    this.drive = m_drive;
    this.poseEstimator = pe;
    this.pathName = name;
    this.constraints = consts;

  }

  @Override
  public void initialize() {
    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, constraints);
    if (path == null) {
      end(false);
      return;
    }

    Alliance alliance = Alliance.Blue;

    PathPlannerTrajectory alliancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);

    // controllerCommand = drive.followTrajectory(driveSystem, poseEstimatorSystem,
    // alliancePath);
    // make FollowTrajectory class
    controllerCommand.initialize();
  }

  @Override
  public void execute() {
    controllerCommand.execute();
  }
}
