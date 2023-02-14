/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.subsystems.Limelight;

public class drivetrain extends SubsystemBase {

	;
	// define Spark Maxes with IDs and as brushless controllers
	public final CANSparkMax backLeft = new CANSparkMax(Constants.BACK_LEFT_SPARKMAX_ID, MotorType.kBrushless);
	public final CANSparkMax frontLeft = new CANSparkMax(Constants.FRONT_LEFT_SPARKMAX_ID, MotorType.kBrushless);
	public final CANSparkMax backRight = new CANSparkMax(Constants.BACK_RIGHT_SPARKMAX_ID, MotorType.kBrushless);
	public final CANSparkMax frontRight = new CANSparkMax(Constants.FRONT_RIGHT_SPARKMAX_ID, MotorType.kBrushless);
	public RelativeEncoder encoder;
	private final AHRS gyro;
	public Robot robot = new Robot();

	// define left and right side controller groups
	MotorControllerGroup m_left = new MotorControllerGroup(frontLeft, backLeft);
	MotorControllerGroup m_right = new MotorControllerGroup(frontRight, backRight);

	// define drive
	private final DifferentialDrive drive = new DifferentialDrive(m_left, m_right);

	public drivetrain() {
		backLeft.setOpenLoopRampRate(.2);
		frontLeft.setOpenLoopRampRate(.2);
		backRight.setOpenLoopRampRate(.2);
		frontRight.setOpenLoopRampRate(.2);

		encoder = backLeft.getEncoder();

		gyro = new AHRS(Port.kMXP);

		// Resets encoder in case counting has already begun.

	}

	public void arcadeDrive(double fwd, double rotation) {
		drive.arcadeDrive(fwd, -rotation);
	}

	// Getter functions
	public Encoder getBackRightEncoder() {
		return (Encoder) backRight.getEncoder();
	}

	public Encoder getBackLeftEncoder() {
		return (Encoder) backLeft.getEncoder();
	}

	public final AHRS getGyro() {
		return gyro;
	}

	public Encoder getFrontRightEncoder() {
		return (Encoder) frontRight.getEncoder();
	}

	public Encoder getFrontLeftEncoder() {
		return (Encoder) frontLeft.getEncoder();
	}

	public class WheelSpeeds {
	}

	public double leftWheelSpeed() {
		return getBackLeftEncoder().getRate();
	}

	public double rightWheelSpeed() {
		return getBackRightEncoder().getRate();
	}

	public void encoderDrive(double dist) {
		while (encoder.getPosition() < 50) {
			arcadeDrive(0, 0.25);
		}
	}

	public void encoderDockDrive(double llY) {
		while (llY != Constants.YOffset) {
			arcadeDrive(0, 0.1);
		}
	}

	public void curvatureDrive(double fwd, double rot, boolean isQuickTurn) {
		drive.curvatureDrive(fwd, rot, isQuickTurn);
	}

	public void setMaxOutput(double maxOutput) {
		drive.setMaxOutput(maxOutput);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public Supplier<Pose2d> getPose() {
		return () -> new DifferentialDriveOdometry(null, robot.encoderBackLeft.getPosition(),
				robot.encoderBackRight.getPosition()).getPoseMeters();
		// TODO: Test these values out
	}

}
