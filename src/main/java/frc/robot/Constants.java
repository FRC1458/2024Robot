package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public final class Constants {

    //Swerve - Auto Constants
    public static final double MAX_VEL_AUTO = 0.85;
    public static final double MAX_ACCEL_AUTO = 0.75;

    //Swerve - Teleop Constants
    public static final double MAX_VEL_TELEOP = 0.75;
    public static final double MAX_ACCEL_TELEOP = 0.6;

    //Swerve - Wheel Locations
    public static final Translation2d FL_WHEEL_LOC = new Translation2d(0, 0);
    public static final Translation2d FR_WHEEL_LOC = new Translation2d(0, 0);
    public static final Translation2d BL_WHEEL_LOC = new Translation2d(0, 0);
    public static final Translation2d BR_WHEEL_LOC = new Translation2d(0, 0);

    //Wheel - Wheels
    public static enum WHEEL {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    //Wheel - Drive Sparks
    public static final int FL_DRIVE_SPARK_ID = 1;
    public static final int FR_DRIVE_SPARK_ID = 2;
    public static final int BL_DRIVE_SPARK_ID = 3;
    public static final int BR_DRIVE_SPARK_ID = 4;

    //Wheel - Angle Sparks
    public static final int FL_ANGLE_SPARK_ID = 11;
    public static final int FR_ANGLE_SPARK_ID = 12;
    public static final int BL_ANGLE_SPARK_ID = 13;
    public static final int BR_ANGLE_SPARK_ID = 14;

    //Wheel - Talons
    public static final int FL_TALON_ID = 1;
    public static final int FR_TALON_ID = 2;
    public static final int BL_TALON_ID = 3;
    public static final int BR_TALON_ID = 4;

    //Wheel - Drive Feedforward Constants
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0;
    public static final double DRIVE_KA = 0;

    //Wheel - Angle PID Constants
    public static final double ANGLE_KP = 0;
    public static final double ANGLE_KI = 0;
    public static final double ANGLE_KD = 0;
    public static final double ANGLE_KFF = 0;
    public static final double ANGLE_IZONE = 0;
    public static final double ANGLE_DFILTER = 0;

}