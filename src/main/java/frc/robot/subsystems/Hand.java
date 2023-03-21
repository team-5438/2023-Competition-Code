package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Hand extends PIDSubsystem {
	//mode 0 is cone, mode 1 is cube

  private final CANSparkMax m_handleftMotor = new CANSparkMax(Constants.HANDLEFT_MOTOR_SPARKMAX_ID,MotorType.kBrushless);
  private final CANSparkMax m_handrightMotor = new CANSparkMax(Constants.HANDRIGHT_MOTOR_SPARKMAX_ID,MotorType.kBrushless);

	private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);

	private final CANSparkMax m_handwristMotor = new CANSparkMax(Constants.WRIST_MOTOR, MotorType.kBrushless);
	private final MotorControllerGroup m_hand = new MotorControllerGroup(m_handleftMotor, m_handrightMotor);
	private Compressor phCompressor = new Compressor(10,PneumaticsModuleType.REVPH); 
	DoubleSolenoid doublePH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

	ArmFeedforward aff = new ArmFeedforward(0,Constants.WristkG,Constants.WristkV, Constants.WristkA);
	
	public Hand(PIDController controller) {
		super(controller);
		phCompressor.enableHybrid(80,110);

		doublePH.set(kOff);
		doublePH.set(kReverse);
	}

	public void solenoidToggle() {
		doublePH.toggle();
	}

	public void handPull(boolean mode) {
		if (mode) {
			//doublePH.set(kForward);
			for(int i = 0; i < 3; i++) {
				m_handleftMotor.set(-0.25);
				m_handrightMotor.set(0.25);
				Timer.delay(1);
			}
		} else {
			//doublePH.set(kReverse);
			for(int i = 0; i < 3; i++) {
				m_handleftMotor.set(-0.25);
				m_handleftMotor.set(0.25);
				Timer.delay(1);
			}
		}
	}
  // 1 = forward, -1 = backwards (probably i have no clue)
	
	public void handRelease(boolean mode) {
		if (!mode) {
			// doublePH.set(kReverse);
			Timer.delay(2);
			// doublePH.set(kForward);
		}
		m_hand.set(0);

		if (mode) {
			for (int i = 0; i < 3; i++) {
				m_handleftMotor.set(0.25);
				m_handrightMotor.set(-0.25);
				Timer.delay(1);
			}
			m_hand.set(0);
		}
	}

	public void moveWrist(double speed) {
		m_handwristMotor.set(MathUtil.applyDeadband(speed, 0.05)); // STOP CHANGING IT!!!
	}

	public void viewPressure() {
		SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		double voltage = getController().calculate(output, setpoint)+aff.calculate(setpoint, 0);
		m_handwristMotor.setVoltage(voltage);
	}

	@Override
	protected double getMeasurement() {
		// TODO Auto-generated method stub
		return wristEncoder.getDistance();
	}
}
