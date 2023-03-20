/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // motor controller ids
  public static final int BACK_LEFT_SPARKMAX_ID = 1;
  public static final int FRONT_LEFT_SPARKMAX_ID = 2;
  public static final int BACK_RIGHT_SPARKMAX_ID = 3;
  public static final int FRONT_RIGHT_SPARKMAX_ID = 4;
  public static final int PIVOT_MOTOR_SPARKMAX_ID = 5;
  public static final int EXTENDER_MOTOR_SPARKMAX_ID = 6;
  public static final int HANDLEFT_MOTOR_SPARKMAX_ID = 7; // ITS CALLED HAND, CHANGE THIS AND I WILL KILL YOU - love, fedor <3
  public static final int HANDRIGHT_MOTOR_SPARKMAX_ID = 8;
  public static final int WRIST_MOTOR = 9;

  public static final class ArmConstants {
    public static final int kMotorPort = 4;

    public static final double kP = 1;

    // These are fake gains; in actuality these must be determined individually for each robot

    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;

    public static final int kEncoderPort = 0;
    public static final int kEncoderPPR = 256;
    public static final double kEncoderDistancePerPulse = 2.0 * Math.PI / kEncoderPPR;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 0.5;
  }


	// controller ids
	public static final int DRIVER_CONTROLLER_PORT = 3;
	public static final int OPERATOR_CONTROLLER_PORT = 4;
	public static final double XOffset = 0;
	public static final double YOffset = 12;

	public static final double kTestAprilTagHeight = 22.9;
	public static final double kRobotHeight = 43;


  // sensor ids
  public static final int LEFT_LIMIT_ID = 1;
  public static final int RIGHT_LIMIT_ID = 0;
  // Other fedor-esque stuff
  public static final double ArmkS = 0;
  public static final double ArmkG =  1.1;
  public static final double ArmkV = 4.19;
  public static final double ArmkA = 0.08;

  public static final double ArmkP = 0.1;
  public static final double ArmkI = 0.001;
  public static final double ArmkD = 0.05;

  public static final double WristP = 0;
  public static final double WristI = 0;
  public static final double WristD = 0;

  public static final double WristkG = 0.3;
  public static final double WristkV = 1.71;
  public static final double WristkA = 0.02;
  
  public static final double ExtenderkG = 1.3;
  public static final double ExtenderkV = 2.3;
  public static final double ExtenderkA = 0.1;

  public static final double kMaxSpeedMetersPerSecond = 0.75;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0.75;

  public static final double kRamseteB = 0.125;
  public static final double kRamseteZeta = 0.85;

  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.5588);
  public static boolean kGyroReversed;
  public static final double kLimelightHeight = 1; // TODO: measure height of limelight
  public static final int SS1PWM = 0;
  public static final int SSLEN = 56;
}
