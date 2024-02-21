package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    double x, y, area, id;
    double[] botpose;

    public Limelight() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry bp = table.getEntry("botpose");
        

        //read values periodically
        
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        id = table.getEntry("tid").getDouble(0.0);
        botpose = bp.getDoubleArray(new double[6]);
    }

    public void displaySmartDashboard() {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getArea() {
        return this.area;
    }

    public double getIDOfAprilTag() {
        return id;
    }

    public Pose2d getBotPosition() {
        return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees((botpose[5])));
    }

}   
