package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.kauailabs.navx.frc.AHRS;
import java.util.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.*;


// 
public class AutoAlign
{
  public AutonomousDrivetrain autoDrive;
  public drivetrain drive;
  public AHRS gyro;
  public Limelight limelight;
  double theta;
  double distance;
  double height;

  Translation2d[] points;

  public AutoAlign()
  {
    gyro = drive.getGyro();
  }
  
  public void Align()
  {
    // height = limelight. getHeight()
    /*1. FIND APRIL TAG PLEASE. angle = theta x
      2. TURN BY theta x from global
      3. OBTAIN theta y; do height * tan(theta y) and subtract a little bit*/
    theta = 90 - gyro.getAngle();
    distance = height * Math.tan(theta);
    
  }
  
}