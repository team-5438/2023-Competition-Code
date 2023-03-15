/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonomousDrivetrain;
import edu.wpi.first.apriltag.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm;
import frc.robot.commands.*;
import edu.wpi.first.math.controller.PIDController;

import java.io.Console;

import com.fasterxml.jackson.databind.deser.AbstractDeserializer;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;

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
  PIDController myPid = new PIDController(0.1, 0.05, 0.01); // Constants: Tune these values
  PIDSendable myPidSendable = new PIDSendable("My PID", myPid);
	private RobotContainer m_robotContainer;
  

	public RelativeEncoder encoderBackLeft;
	public RelativeEncoder encoderBackRight;
	public RelativeEncoder encoderFrontLeft;
	public RelativeEncoder encoderFrontRight;
	public Limelight limelight;
	public Arm arm;

	public AutonomousDrivetrain autodrive;
  public drivetrain drive;
  public AHRS gyro;

	public double currentVoltage;
	public double[] voltages;
	int num;

  public Robot() 
  {
    
  }

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
		//arm = new Arm();

		m_robotContainer = new RobotContainer(limelight);
		limelight.setDrivetrain(m_robotContainer.m_drivetrain);

		encoderBackLeft = m_robotContainer.m_drivetrain.backLeft.getEncoder();
		encoderBackRight = m_robotContainer.m_drivetrain.backRight.getEncoder();
		encoderFrontLeft = m_robotContainer.m_drivetrain.frontLeft.getEncoder();
		encoderFrontRight = m_robotContainer.m_drivetrain.frontRight.getEncoder();

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

		voltages = new double[100];
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
    // place object on L3 height
    while (gyro.getAngle() < 180)
    {
      
    }
    // drive over charging stand
    // go back behind
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

		m_robotContainer.arm.pivotArm(m_robotContainer.getPivotSpeed());

		m_robotContainer.arm.extendArm(m_robotContainer.getExtenderSpeed()); 

		m_robotContainer.hand.moveWrist(m_robotContainer.getWristSpeed());

		if (m_robotContainer.operatorController.getStartButtonPressed()){
			m_robotContainer.changeMode();
		}

		if(m_robotContainer.operatorController.getYButtonPressed()){
			m_robotContainer.hand.handRelease(m_robotContainer.cubeMode);
		}

		if(m_robotContainer.operatorController.getXButton()){
			m_robotContainer.hand.handPull(m_robotContainer.cubeMode);
		}

		SmartDashboard.putBoolean("Mode", m_robotContainer.cubeMode);

		
	}
	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
    // PID tune wrist & arm
		CommandScheduler.getInstance().run();
/*    LiveWindow lw = LiveWindow.getInstance();
    DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);
    DutyCycleEncoder armmEncoder = new DutyCycleEncoder();
    lw.addSensor("Wrist", "Encoder", wristEncoder);
    lw.addSensor("Arm", "Encoder", armEncoder);

    CANSparkMax wristMotor = new CANSparkMax(9):
    lw.addActuator("Wrist", "Motor", wristMotor);
    CANSParkMax armMotor = new CANSParkMax();
    lw.addActuator("Arm", "Motor", armMotor);

  */  
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
    SmartDashboard.putData(myPidSendable);
	}
}
