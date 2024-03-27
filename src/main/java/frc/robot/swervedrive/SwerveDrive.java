package frc.robot.swervedrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class SwerveDrive {
    ChassisSpeeds speeds;
    public final Wheel frontLeft;
    public final Wheel frontRight;
    public final Wheel backLeft;
    public final Wheel backRight;
    SwerveDriveKinematics kinematics;
    //SwerveDriveOdometry odometry;
    SwerveDrivePoseEstimator odometry;
    private final AHRS navX;

    Translation2d frontLeftLocation;
    Translation2d frontRightLocation;
    Translation2d backLeftLocation;
    Translation2d backRightLocation;

    SwerveModulePosition frontLeftPosition;
    SwerveModulePosition frontRightPosition;
    SwerveModulePosition backLeftPosition;
    SwerveModulePosition backRightPosition;

    private double maxVelocity = -1;

    public SwerveDrive(AHRS navX) {
        this.navX = navX;
        frontLeft = new Wheel(RobotConstants.frontLeftAngleID, RobotConstants.frontLeftSpeedID, RobotConstants.frontLeftAbsoluteEncoderID, "Front Left (1)", RobotConstants.frontLeftAngleOffset);
        frontRight = new Wheel(RobotConstants.frontRightAngleID, RobotConstants.frontRightSpeedID, RobotConstants.frontRightAbsoluteEncoderID, "Front Right (2)", RobotConstants.frontRightAngleOffset);
        backLeft = new Wheel(RobotConstants.backLeftAngleID, RobotConstants.backLeftSpeedID, RobotConstants.backLeftAbsoluteEncoderID, "Back Left (3)", RobotConstants.backLeftAngleOffset);
        backRight = new Wheel(RobotConstants.backRightAngleID, RobotConstants.backRightSpeedID, RobotConstants.backRightAbsoluteEncoderID, "Back Right (4)", RobotConstants.backRightAngleOffset);

        SmartDashboard.putNumber("SwerveKP: ", 0.0001);
        SmartDashboard.putNumber("SwerveKI: ", 0.00001);
        SmartDashboard.putNumber("SwerveKD: ", 0.0001);
        SmartDashboard.putNumber("Swerve iScaling: ", 20);
        
        frontLeft.setPID(0.0001, 0.00001, 0.0001, 20);
        frontRight.setPID(0.0001, 0.00001, 0.0001, 20);
        backRight.setPID(0.0001, 0.00001, 0.0001, 20);
        backLeft.setPID(0.0001, 0.00001, 0.0001, 20);

        frontLeftLocation = new Translation2d(RobotConstants.frontLeftXMeters, RobotConstants.frontLeftYMeters);
        frontRightLocation = new Translation2d(RobotConstants.frontRightXMeters, RobotConstants.frontRightYMeters);
        backLeftLocation = new Translation2d(RobotConstants.backLeftXMeters, RobotConstants.backLeftYMeters);
        backRightLocation = new Translation2d(RobotConstants.backRightXMeters, RobotConstants.backRightYMeters);

        frontLeftPosition = new SwerveModulePosition(RobotConstants.frontLeftDistance, RobotConstants.frontLeftAngle);
        frontRightPosition = new SwerveModulePosition(RobotConstants.frontRightDistance, RobotConstants.frontRightAngle);
        backLeftPosition = new SwerveModulePosition(RobotConstants.backLeftDistance, RobotConstants.backLeftAngle);
        backRightPosition = new SwerveModulePosition(RobotConstants.backRightDistance, RobotConstants.backRightAngle);
        SwerveModulePosition[] positions = {frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition};

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        //odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions, new Pose2d(5.0, 13.5, new Rotation2d()));
        //odometry not used??? also not assigned properly? check w/mentor before using
        odometry = new SwerveDrivePoseEstimator(kinematics,
                Rotation2d.fromDegrees(navX.getAngle()),
                positions,
                new Pose2d(RobotConstants.initialXPos, RobotConstants.initialYPos, Rotation2d.fromDegrees(navX.getAngle())),
                VecBuilder.fill(RobotConstants.PositionStdDevX, RobotConstants.PositionStdDevY, Units.degreesToRadians(RobotConstants.PositionStdDevTheta)),
                VecBuilder.fill(RobotConstants.LimelightStdDevX, RobotConstants.LimelightStdDevY, Units.degreesToRadians(RobotConstants.LimelightStdDevTheta)));
        speeds = new ChassisSpeeds();


    }

    public void drive(double x, double y, double r, boolean fieldOriented) {

        SmartDashboard.putNumber("Gyro Angle", navX.getAngle());

        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = r;

        double KP = SmartDashboard.getNumber("SwerveKP: ", 0.0001);
        double KI = SmartDashboard.getNumber("SwerveKI: ", 0);
        double KD = SmartDashboard.getNumber("SwerveKD: ", 0.0001);
        double iScaling = SmartDashboard.getNumber("Swerve iScaling: ", 20);




        frontLeft.setPID(KP, KI, KD, iScaling);
        frontRight.setPID(KP, KI, KD, iScaling);
        backRight.setPID(KP, KI, KD, iScaling);
        backLeft.setPID(KP, KI, KD, iScaling);

        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, Rotation2d.fromDegrees(-(navX.getYaw())));
        }
        //SmartDashboard.putNumber("angle from navx", navX.getYaw());

        //SmartDashboard.putNumber("X", x);
        //SmartDashboard.putNumber("Y", y);
        //SmartDashboard.putNumber("R", r);
        //SmartDashboard.putNumber("Robot Angle", navX.getYaw());
        //SmartDashboard.putNumber("Robot Angle (pitch)", navX.getPitch());

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeft.drive(states[0].speedMetersPerSecond, states[0].angle.getDegrees());
        frontRight.drive(states[1].speedMetersPerSecond, states[1].angle.getDegrees());
        backLeft.drive(states[2].speedMetersPerSecond, states[2].angle.getDegrees());
        backRight.drive(states[3].speedMetersPerSecond, states[3].angle.getDegrees());

        double absVelocity = Math.abs(frontLeft.getVelocity());
        if(absVelocity > maxVelocity){
            maxVelocity = absVelocity;
        }
        SmartDashboard.putNumber("(FL) Max Velocity: ", maxVelocity);
        SmartDashboard.putNumber("(FL) Abs Enc Val", frontLeft.getAbsoluteEncoderValue());

    }

    //delete this?
    public double turnToAngle(double goalAngle, double angle) {
        double error = 2.0;

        double diff = (angle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
            diff = diff - 360 * Math.signum(diff);
        }

        double realGoalAngle = (angle - diff);

        if (Math.abs(angle - realGoalAngle) > error) {
            if (angle > realGoalAngle) {
                return -.1;
            } else {
                return .1;
            }
        }
        return 0;

    }

    public void setEncoders() {
        frontLeft.setEncoders(RobotConstants.frontLeftAngleOffset);
        frontRight.setEncoders(RobotConstants.frontRightAngleOffset);
        backLeft.setEncoders(RobotConstants.backLeftAngleOffset);
        backRight.setEncoders(RobotConstants.backRightAngleOffset);
    }

    public void resetNavX() {
        navX.reset();
    }

    public void resetMaxVel() {
        maxVelocity = -1;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    public Rotation2d navxAngle() {
        return Rotation2d.fromDegrees(navX.getAngle());
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void displayPositions() {
        frontLeft.displayPosition();
        frontRight.displayPosition();
        backLeft.displayPosition();
        backRight.displayPosition();
    }

    public Pose2d updateOdometry() {
        return odometry.update(Rotation2d.fromDegrees(navX.getAngle()), getPositions());
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        frontLeft.resetOdometry();
        frontRight.resetOdometry();
        backLeft.resetOdometry();
        backRight.resetOdometry();
        odometry.resetPosition(new Rotation2d().minus(navX.getRotation2d()), getPositions(), pose);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

}