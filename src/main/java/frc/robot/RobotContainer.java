/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import java.awt.Robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutonomousDrivetrain;
// subsystems
import frc.robot.subsystems.*;
// commands
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
	XboxController driveController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
	XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

	// define subsystems
	drivetrain m_drivetrain = new drivetrain();
	AutonomousDrivetrain autodrive;

	Limelight limelight;

	Arm arm = new Arm(null);
	Hand hand = new Hand(new PIDController(Constants.WristP, Constants.WristI, Constants.WristD));

	AddressableLED sponsorStrip1;
	AddressableLED sponsorStrip2;
	AddressableLED electronicsStrip;

	AddressableLEDBuffer sponsorStrip1Buffer;
	AddressableLEDBuffer sponsorStrip2Buffer;
	AddressableLEDBuffer electronicsStripBuffer;
	
	public boolean cubeMode = false; //0 is cone, 1 is cube

	// The container for the robot. Contains subsystems, OI devices, and commands.
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

		sponsorStrip1 = new AddressableLED(Constants.SS1PWM);

		sponsorStrip1Buffer = new AddressableLEDBuffer(Constants.SSLEN);

		sponsorStrip1.setLength(sponsorStrip1Buffer.getLength());

		sponsorStrip1.setData(sponsorStrip1Buffer);

		sponsorStrip1.start();
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
		return driveController.getLeftY();
	}

	public double getTurnAxis() {
		return driveController.getRightX();
	}

	public double getPivotSpeed() {
		return operatorController.getLeftY();
	}

	public double getExtenderSpeed() {
		double extend = 0;
		if(extend == 0 && operatorController.getLeftTriggerAxis() > 0){
			extend = operatorController.getLeftTriggerAxis();
		}

		else if (extend == 0 && operatorController.getRightTriggerAxis() > 0){
			extend = operatorController.getRightTriggerAxis();
		}

		return extend;
	}

	public double getWristSpeed() {
		return operatorController.getRightY();
	}

	public void setStripColor(AddressableLED m_led, AddressableLEDBuffer m_ledbuffer,int r, int g, int b) {
		for (int i = 0; i < m_ledbuffer.getLength(); i++) {
			m_ledbuffer.setRGB(i, r, g, b);
		}

		m_led.setData(m_ledbuffer);
	}

	public void setLED() {
		// setStripColor(electronicsStrip, electronicsStripBuffer, 255, 255, 0);
		if(cubeMode) {
		setStripColor(sponsorStrip1, sponsorStrip1Buffer, 255, 0, 255);
		}
		else{
			setStripColor(sponsorStrip1, sponsorStrip1Buffer, 255, 128, 0);
		}
		// setStripColor(sponsorStrip2, sponsorStrip2Buffer, 255, 255, 0);
	}

	public void changeMode() {
		cubeMode = !cubeMode;
		setLED();
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
