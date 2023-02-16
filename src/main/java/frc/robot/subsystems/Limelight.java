package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.subsystems.*;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import org.photonvision.PhotonCamera;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.drivetrain; 


public class Limelight {
	public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	public NetworkTableEntry tx = table.getEntry("tx");
	public NetworkTableEntry ty = table.getEntry("ty");
	public NetworkTableEntry ta = table.getEntry("ta");
	public double z;
	public double x;
	public double y;
	public double area;

    public RelativeEncoder leftEncoder;
    public RelativeEncoder rightEncoder;
    public drivetrain dtrain;
    public AHRS gyro;

    public Limelight(drivetrain drive){
        dtrain = drive;
        leftEncoder = drive.leftEncoder;
        rightEncoder = drive.rightEncoder;

        gyro = drive.getGyro();
    }

	final public void getValues() {
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		// read values periodically
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		z = Math.sqrt((x * x) + (y * y));
		// TODO: Change x, y, and z to lengths instead of angle measures

		// post to smart dashboard periodically
		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
	}

	public void setPipeline(int pipeline) {
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
		pipelineEntry.setNumber(pipeline);
	}

	final public double getDistance() {
		double distance = (Constants.kTestAprilTagHeight - Constants.kRobotHeight) / Math.tan(y);
		return distance;
	}

	final public Supplier<Pose2d> getPose() {
	 double[] poseArray = table.getEntry("botpose").getDoubleArray(new double[6]);
	 return () -> new DifferentialDriveOdometry(gyro.getRotation2d(),leftEncoder.getPosition(),rightEncoder.getPosition()).getPoseMeters();
	 // TODO: Test these values out
	 }
}
