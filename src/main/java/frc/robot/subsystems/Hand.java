package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;

public class Hand extends PIDSubsystem {
	public Hand(PIDController controller) {
		super(controller);
	}

	static private final CANSparkMax m_handleftMotor = new CANSparkMax(Constants.HANDLEFT_MOTOR_SPARKMAX_ID,MotorType.kBrushless);
    static private final CANSparkMax m_handrightMotor = new CANSparkMax(Constants.HANDRIGHT_MOTOR_SPARKMAX_ID,MotorType.kBrushless);

	private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          Constants.ksVolts, Constants.kvVoltSecondsPerMeter);

	static private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);
	//static private DigitalInput topLimitSwitch = new DigitalInput(0);
	//static private DigitalInput bottomLimitSwitch = new DigitalInput(0);

	public static void handPull(double speed) {
			m_handleftMotor.set(MathUtil.applyDeadband(speed, 0.04));
			m_handrightMotor.set(MathUtil.applyDeadband(speed, 0.04));
		// 1 = forward, -1 = backwards (probably i have no clue)
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected double getMeasurement() {
		// TODO Auto-generated method stub
		return 0;
	}
}
