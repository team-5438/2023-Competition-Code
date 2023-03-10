// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import frc.robot.Constants;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.*;

// public class Arm extends PIDSubsystem {

// 	static private final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_SPARKMAX_ID, MotorType.kBrushless);
// 	static private final CANSparkMax extenderMotor = new CANSparkMax(Constants.EXTENDER_MOTOR_SPARKMAX_ID, MotorType.kBrushless);

// 	static private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

// 	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter);
	
// 	public Arm(PIDController controller) {
// 		super(controller);

// 		getController().setIntegratorRange(-0.5, 0.5);
// 		getController().setTolerance(1, 1);
// 	}
// 	//static private DigitalInput topLimitSwitch = new DigitalInput(0);
// 	//static private DigitalInput bottomLimitSwitch = new DigitalInput(0);

// 	public void pivotArm(double speed) {
// 			pivotMotor.set(MathUtil.applyDeadband(speed, 0.02));

// 		}
// 		// 1 = forward, -1 = backwards
	

// 	public void extendArm(double speed){
// 		extenderMotor.set(MathUtil.applyDeadband(speed, 0.05));
// 	}

// 	@Override
// 	protected void useOutput(double output, double setpoint) {
// 		pivotMotor.set(getController().calculate(output,setpoint));
		
// 	}

// 	@Override
// 	protected double getMeasurement() {
// 		// TODO Auto-generated method stub
// 		return 0;
// 	}
// }
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class Arm extends ProfiledPIDSubsystem {
  private final CANSparkMax pivot_motor = new CANSparkMax(Constants.PIVOT_MOTOR_SPARKMAX_ID, MotorType.kBrushless);

  private final CANSparkMax extender_motor = new CANSparkMax(Constants.EXTENDER_MOTOR_SPARKMAX_ID, MotorType.kBrushless);

  private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kEncoderPort);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(Constants.ArmkS, Constants.ArmkG, Constants.ArmkV, Constants.ArmkA);

  /** Create a new ArmSubsystem. */
  public Arm() {
    super(new ProfiledPIDController( ArmConstants.kP, 0,0, new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocityRadPerSecond,																																														ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);

    // Start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);
  }

  public void pivotArm(double speed){
	pivot_motor.set(MathUtil.applyDeadband(speed, 0.05));
  }

  public void extendArm(double speed){
	extender_motor.set(MathUtil.applyDeadband(speed, 0.05));
  }

  

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    pivot_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
  }
}