package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Intake extends CommandBase {
  public final CANSparkMax LeftMotor = new CANSparkMax(Constants.kLeftIntakeMotor);
  public final CANSparkMax RightMotor = new CANSparkMax(Constants.kRightIntakeMotor);
  Joystick controller = new Joystick(Constants.DRIVER_CONTROLLER_PORT);

  public void IntakeCommand()
  {
    
  }
}