package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {
    //swerve stuff
    public final static int frontLeftAngleID = 19;
    public final static int frontRightAngleID = 15;
    public final static int backLeftAngleID = 13;
    public final static int backRightAngleID = 4;
    
    public final static int frontLeftSpeedID = 9;
    public final static int frontRightSpeedID = 5;
    public final static int backLeftSpeedID = 3;
    public final static int backRightSpeedID = 7;
    
    public final static double frontLeftAngleOffset = -5.05;
    public final static double frontRightAngleOffset = 4.4;
    public final static double backLeftAngleOffset = 0.5;
    public final static double backRightAngleOffset = -.15;

    public final static double kP = 0.1;
    public final static double kI = 0.000001;
    public final static double kD = 0.000001;

    public final static double frontLeftXMeters = -0.2667; //wheel offsets are 10.5 inches
    public final static double frontLeftYMeters = 0.2667;
    public final static double frontRightXMeters = 0.2667;
    public final static double frontRightYMeters = 0.2667;
    public final static double backLeftXMeters = -0.2667;
    public final static double backLeftYMeters = -0.2667;
    public final static double backRightXMeters = 0.2667;
    public final static double backRightYMeters = -0.2667;

    public final static double swerveDriveGearRatio = 12.8;

    public final static double speed = 1;

    public final static int frontLeftAbsoluteEncoderID = 13;//13?
    public final static int frontRightAbsoluteEncoderID = 26;//26?
    public final static int backLeftAbsoluteEncoderID = 11;//???
    public final static int backRightAbsoluteEncoderID = 15;//???

    // CHANGE ALL THESE VALUES TO REAL NUMBERS
    public final static double frontLeftDistance = 0;
    public final static double frontRightDistance = 0;
    public final static double backLeftDistance = 0;
    public final static double backRightDistance = 0;
    
    public final static Rotation2d frontLeftAngle = new Rotation2d(0);
    public final static Rotation2d frontRightAngle = new Rotation2d(0);
    public final static Rotation2d backLeftAngle = new Rotation2d(0);
    public final static Rotation2d backRightAngle = new Rotation2d(0);

    public final static double maxSwerveSpeed = 6200;

    public final static int intakeMotorID = 30;
    public final static int plumbingMotorID = 31;
    public final static int rightShooterMotorID = 32;
    public final static int leftShooterMotorID = 33;

    public final static double intakeMotorSpeed = 0.25; //speeds between 0 and 1
    public final static double plumbingMotorSpeed = 0.25; //can be changed, can also possibly just use intakeMotorSpeed for plumbing motor as well
    public final static double shooterSpeed = 0.75;

    public final static double shooterRampUpTime = 1; //in seconds
}
