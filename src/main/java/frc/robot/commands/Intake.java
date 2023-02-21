package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// TODO: Get current voltage at set rate in Robot.java
// push values to an array with n values.
// When array fills, delete values and set mean as first value
// get the mean of all values
// check to see if a voltage is greater than the mean

// This class should be responsible for taking in objects and releasing them.
public class Intake extends CommandBase {
  public final CANSparkMax LeftMotor = new CANSparkMax(Constants.kLeftIntakeMotor, MotorType.kBrushless);
  public final CANSparkMax RightMotor = new CANSparkMax(Constants.kRightIntakeMotor, MotorType.kBrushless);

  public Robot robot = new Robot();
  
  public void setNumber(double input)
  {
    
  }
  
  public final XboxController controller = new 
  XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  public MotorControllerGroup motorControllerGroup = new MotorControllerGroup(LeftMotor, RightMotor);
  public void IntakeCommand()
  {
    if (controller.getBButtonPressed())
    {
      while ((robot.currentVoltage) < 3)
      {
        LeftMotor.set(-0.65);
        RightMotor.set(-0.65); // placeholders
      }
    }
  }

  public void ReleaseCommand()
  {
    
  }
}