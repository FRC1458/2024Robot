package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {
    //swerve stuff
    public final static int frontLeftAngleID = 17;
    public final static int frontRightAngleID = 13;
    public final static int backLeftAngleID = 15;
    public final static int backRightAngleID = 19;
    
    public final static int frontLeftSpeedID = 7;
    public final static int frontRightSpeedID = 3;
    public final static int backLeftSpeedID = 5;
    public final static int backRightSpeedID = 9;
    
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

    public final static int frontLeftAbsoluteEncoderID = 27;
    public final static int frontRightAbsoluteEncoderID = 23;
    public final static int backLeftAbsoluteEncoderID = 25;
    public final static int backRightAbsoluteEncoderID = 29;

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

    public final static int intakeMotorID = 33;
    public final static int feederMotorID = 34;
    public final static int rightShooterMotorID = 31;
    public final static int leftShooterMotorID = 32;

    public final static double intakeMotorSpeed = 0.25; //speeds between 0 and 1
    public final static double feederMotorSpeed = 0.25; //can be changed, can also possibly just use intakeMotorSpeed for feeder motor as well
    public final static double shooterSpeed = 0.75;
    public final static double shooterPIDSpeed = 30;
    

    public final static double shooterRampUpTime = 1; //in seconds

    //can be changed later to dynamically choose based on auto
    public final static double initialXPos = 0;
    public final static double initialYPos = 0;

    //maybe change
    //bigger number = less trusting of model state estimate
    public final static double PositionStdDevX = 0.1;
    public final static double PositionStdDevY = 0.1;
    public final static double PositionStdDevTheta = 10;

    public final static double LimelightStdDevX = 5;
    public final static double LimelightStdDevY = 5;
    public final static double LimelightStdDevTheta = 500;
}
