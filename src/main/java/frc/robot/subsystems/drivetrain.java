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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;

public class drivetrain extends SubsystemBase {

	
	;
	// define Spark Maxes with IDs and as brushless controllers
	public final CANSparkMax backLeft = new CANSparkMax(Constants.BACK_LEFT_SPARKMAX_ID, MotorType.kBrushless);
	public final CANSparkMax frontLeft = new CANSparkMax(Constants.FRONT_LEFT_SPARKMAX_ID, MotorType.kBrushless);
	public final CANSparkMax backRight = new CANSparkMax(Constants.BACK_RIGHT_SPARKMAX_ID, MotorType.kBrushless);
	public final CANSparkMax frontRight = new CANSparkMax(Constants.FRONT_RIGHT_SPARKMAX_ID, MotorType.kBrushless);
	public RelativeEncoder encoder;

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

		// Resets encoder in case counting has already begun.
	
	}

	public void arcadeDrive(double fwd, double rot) {
		drive.arcadeDrive(fwd, -rot);
	}

	public void encoderDrive(double dist) {
		if (encoder.getPosition() < dist) {
			// Drives forward 5 ft.
			arcadeDrive(0.5, 0);
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
}
