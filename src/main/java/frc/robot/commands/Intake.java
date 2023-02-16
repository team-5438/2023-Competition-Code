package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
// TODO: Get current voltage at set rate in Robot.java
// push values to an array with n values.
// When array fills, delete values and set mean as first value
// get the mean of all values
// check to see if a voltage is greater than the mean

// This class should be responsible for taking in objects and releasing them.
public class Intake extends CommandBase {
  public final CANSparkMax LeftMotor = new CANSparkMax(Constants.kLeftIntakeMotor);
  public final CANSparkMax RightMotor = new CANSparkMax(Constants.kRightIntakeMotor);

  public double previousVoltage = currentVoltage; // replace this with array in Robot.java
  public double currentVoltage = ((LeftMotor.getBusVoltage() + RightMotor.getBusVoltage()) / 2);

  public void setNumber(double input)
  {
    
  }
  
  public final XboxController controller = new 
  XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  public MotorControllerGroup = new MotorControllerGroup(LeftMotor, RightMotor);
  public void IntakeCommand()
  {
    if (controller.getBButtonPressed())
    {
      while ((currentVoltage - previousVoltage) < 3)
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