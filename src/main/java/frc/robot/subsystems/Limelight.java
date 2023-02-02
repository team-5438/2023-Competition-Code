package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    final public void getValues(){
        double testDbl = NetworkTablesJNI.getDouble(301989889, 0);
        SmartDashboard.putNumber("ty", testDbl);
        //double x = tx.getDouble(0.0);
        //double y = ty.getDouble(0.0);
        //double area = ta.getDouble(0.0);

        //SmartDashboard.putNumber("Limelight X", x);
        //SmartDashboard.putNumber("Limelight Y", y);
        //SmartDashboard.putNumber("Limelight Area", area);
    }
    public void setPipeline(int pipeline) {
		NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    	pipelineEntry.setNumber(pipeline);
    }
}
