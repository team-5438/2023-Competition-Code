package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.kauailabs.navx.frc.AHRS;
import java.util.*;

public class AutoAlign
{
  public drivetrain drive;
  public AHRS gyro;
  public Limelight limelight;
  double theta;
  double alpha;
  double distance;
  double height;

  public AutoAlign()
  {
    gyro = drive.getGyro();
  }
  
  public void Align()
  {
    // height = limelight. getHeight()
    alpha = gyro.getAngle(); //Note: Angle can be more than 360 degrees. Try to reduce angle down to <90.
    theta = 90 - alpha;
    distance = height / Math.tan(theta);

    
  }
  
}