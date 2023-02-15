/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.AutonomousDrivetrain;
//subsystems
import frc.robot.subsystems.*;
//commands
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */

public class RobotContainer {
	// define controllers
	Joystick driveController = new Joystick(Constants.DRIVER_CONTROLLER_PORT);

	// define subsystems
	drivetrain m_drivetrain = new drivetrain();

	Limelight limelight;

	Arm arm;

	AutonomousDrivetrain autodrive;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer(Limelight ll) {

		// Configure the button bindings
		configureButtonBindings();

		// set default commands
		m_drivetrain.setDefaultCommand(
				new DefaultDrive(
						m_drivetrain,
						() -> getFwdAxis(),
						() -> getTurnAxis()));

		limelight = ll;

		autodrive = new AutonomousDrivetrain(m_drivetrain, limelight);

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

	}

	// fightstickLBButton.whileHeld(new LimelightTurretAim(m_turret));

	public double getFwdAxis() {
		return driveController.getLeftX();
	}

	public double getTurnAxis() {
		return driveController.getLeftY();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutoAlignCommand() {
		// An ExampleCommand will run in autonomous
		AprilTag apriltag = new AprilTag(1, autodrive.getPose());

		Command autoalign = autodrive.getAutonomousCommand(apriltag);

		return autoalign;

	}
}
