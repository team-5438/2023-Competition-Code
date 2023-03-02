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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Hand extends PIDSubsystem {

	static private final CANSparkMax m_handleftMotor = new CANSparkMax(Constants.HANDLEFT_MOTOR_SPARKMAX_ID,MotorType.kBrushless);
    static private final CANSparkMax m_handrightMotor = new CANSparkMax(Constants.HANDRIGHT_MOTOR_SPARKMAX_ID,MotorType.kBrushless);
	static private final CANSparkMax m_wristMotor = new CANSparkMax(Constants.WRIST_MOTOR, MotorType.kBrushless);

	private Compressor phCompressor = new Compressor(0,PneumaticsModuleType.REVPH); 

	DoubleSolenoid doublePH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);


	private final SimpleMotorFeedforward m_feedforward =new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter);

	//static private DigitalInput topLimitSwitch = new DigitalInput(0);
	//static private DigitalInput bottomLimitSwitch = new DigitalInput(0);

	public Hand(PIDController controller) {
		super(controller);
		phCompressor.enableHybrid(20,40);

		doublePH.set(kOff);
		doublePH.set(kReverse);
	}

	public void solenoidToggle(){
		doublePH.toggle();
	}


	public void handPull(double speed) {
			m_handleftMotor.set(MathUtil.applyDeadband(speed, 0.04));
			m_handrightMotor.set(MathUtil.applyDeadband(speed, 0.04));

			solenoidToggle();
		// 1 = forward, -1 = backwards (probably i have no clue)
	}

	public void moveWrist(double speed){
		m_wristMotor.set(MathUtil.applyDeadband(speed, 0.05));
	}

	public  void viewPressure (){
		SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
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
