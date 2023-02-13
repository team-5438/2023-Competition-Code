package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

public class Limelight {
	public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

	public NetworkTableEntry tx = table.getEntry("tx");
	public NetworkTableEntry ty = table.getEntry("ty");
	public NetworkTableEntry ta = table.getEntry("ta");
	public double z;
	public double x;
	public double y;
	public double area;

	final public void getValues() {
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry ta = table.getEntry("ta");
		// read values periodically
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		z = Math.sqrt((x * x) + (y * y));

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

		return Math.sqrt((x * x) + (y * y) + (z * z));

	}
}
