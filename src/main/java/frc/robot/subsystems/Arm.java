package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_SPARKMAX_ID, MotorType.kBrushless);

	static public void pivotArm(double speed) {
		// 1 = forward, -1 = backwards
		// pivotMotor.set(applyDeadband(speed, 0.02));
	}
}
