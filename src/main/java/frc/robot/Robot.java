/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutonomousDrivetrain;
import edu.wpi.first.apriltag.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm;

import java.io.Console;

import com.fasterxml.jackson.databind.deser.AbstractDeserializer;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	// Initializes encoder
	// TODO: Name encoder as per function <23-01-23, slys> //

	private RobotContainer m_robotContainer;

	public RelativeEncoder encoderBackLeft;
	public RelativeEncoder encoderBackRight;
	public RelativeEncoder encoderFrontLeft;
	public RelativeEncoder encoderFrontRight;
	public Limelight limelight;
	public Arm arm;

	public AutonomousDrivetrain autodrive;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.''
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.

		limelight = new Limelight();
		limelight.getValues();
		limelight.table.getInstance().startServer();
		arm = new Arm();

		m_robotContainer = new RobotContainer(limelight);

		encoderBackLeft = m_robotContainer.m_drivetrain.backLeft.getEncoder();
		encoderBackRight = m_robotContainer.m_drivetrain.backRight.getEncoder();
		encoderFrontLeft = m_robotContainer.m_drivetrain.frontLeft.getEncoder();
		encoderFrontRight = m_robotContainer.m_drivetrain.frontRight.getEncoder();
		limelight = new Limelight();
		limelight.getValues();
		limelight.table.getInstance().startServer();

		autodrive = new AutonomousDrivetrain(m_robotContainer.m_drivetrain, m_robotContainer.limelight);

		// Resets encoder in case counting has already begun.
		encoderBackLeft.setPosition(0);
		encoderBackRight.setPosition(0);
		encoderFrontLeft.setPosition(0);
		encoderFrontRight.setPosition(0);
		// 1ft per rotation (256 rotations)
		encoderBackLeft.setPositionConversionFactor(1. / 256.);
		encoderBackRight.setPositionConversionFactor(1. / 256.);
		encoderFrontLeft.setPositionConversionFactor(1. / 256.);
		encoderFrontRight.setPositionConversionFactor(1. / 256.);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and
	 * test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutoAlignCommand();

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		Arm.pivotArm(0.5);

		// m_robotContainer.m_drivetrain.encoderDockDrive(y);
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		m_robotContainer.m_drivetrain.arcadeDrive(m_robotContainer.getFwdAxis() / 13,
				m_robotContainer.getTurnAxis() / 13);
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
