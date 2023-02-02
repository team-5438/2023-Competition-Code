/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	//motor controller ids
	public static final int BACK_LEFT_SPARKMAX_ID = 1;
	public static final int FRONT_LEFT_SPARKMAX_ID = 2;
	public static final int BACK_RIGHT_SPARKMAX_ID = 3;
	public static final int FRONT_RIGHT_SPARKMAX_ID = 4;
	
	//controller ids
	public static final int DRIVER_CONTROLLER_PORT = 3;
	public static final int FIGHTSTICK_PORT = 1;
	public static final int A_BUTTON_PORT = 1;
	public static final int B_BUTTON_PORT = 2;
	public static final int X_BUTTON_PORT = 3;
	public static final int Y_BUTTON_PORT = 4;
	public static final int LB_BUTTON_PORT = 5;
	public static final int RB_BUTTON_PORT = 6;
	public static final double XOffset = 0;
	public static final double YOffset = 12;

	//sensor ids
	public static final int LEFT_LIMIT_ID = 1;
	public static final int RIGHT_LIMIT_ID = 0;
}