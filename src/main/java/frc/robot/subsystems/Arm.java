package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;

public class Arm extends PIDSubsystem {

	static private final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_SPARKMAX_ID, MotorType.kBrushless);
	static private final CANSparkMax extenderMotor = new CANSparkMax(Constants.EXTENDER_MOTOR_SPARKMAX_ID, MotorType.kBrushless);

	static private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
  static private boolean ArmLimitReached;

	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter);
	
	public Arm(PIDController controller) {
		super(controller);
    pivotEncoder
    pivotEncoder.reset();
		getController().setIntegratorRange(-0.5, 0.5);
		getController().setTolerance(1, 1);
	}
	//static private DigitalInput topLimitSwitch = new DigitalInput(0);
	//static private DigitalInput bottomLimitSwitch = new DigitalInput(0);

	public void pivotArm(double speed) {
			pivotMotor.set(MathUtil.applyDeadband(speed, 0.02));

		}
		// 1 = forward, -1 = backwards
	

	public void extendArm(double speed){
		extenderMotor.set(MathUtil.applyDeadband(speed, 0.05));
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		pivotMotor.set(getController().calculate(output,setpoint));
		
	}

	@Override
	protected double getMeasurement() {
		// TODO Auto-generated method stub
		return 0;
	}
}
