package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;

public class Arm extends SubsystemBase {
	static private final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_SPARKMAX_ID,
			MotorType.kBrushless);
	//static private DigitalInput topLimitSwitch = new DigitalInput(0);
	//static private DigitalInput bottomLimitSwitch = new DigitalInput(0);

	static public void pivotArm(double speed) {
			pivotMotor.set(MathUtil.applyDeadband(speed, 0.02));
		// 1 = forward, -1 = backwards
	}
}
