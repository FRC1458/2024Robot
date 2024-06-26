package frc.robot.swervedrive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;


public class Wheel {
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkPIDController pidController;
    public final RelativeEncoder encoder;
    private final RelativeEncoder driveEncoder;
    private TalonSRX absoluteEncoder;

    private double relativeOffset = 0;

    private double goalAngle;

    private double speed;

    public final String wheelName;
    private double offset;
    private double offset2 = 0;

    private boolean diagnostic;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    PID betterPID = new PID();


    private double WHEEL_RAD = Units.inchesToMeters(2);
    private double ANGLE_TO_DRIVE_RATIO = 3.584;
    private double DRIVE_RATIO = 8.14; // L1

    private RelativeEncoder driveEnc;
    private double angleMarker;


    public Wheel(int angleMotorID, int speedMotorID, int absoluteEncoderID, String wheelName, double offset) {

        this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        this.absoluteEncoder = new TalonSRX(absoluteEncoderID);
        this.offset = offset;

        this.absoluteEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        this.wheelName = wheelName;

        pidController = angleMotor.getPIDController();
        encoder = angleMotor.getEncoder();
        driveEncoder = speedMotor.getEncoder();


        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;//5700

        pidController.setP(RobotConstants.kP);
        pidController.setI(RobotConstants.kI);
        pidController.setD(RobotConstants.kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        

        SmartDashboard.putNumber(wheelName + " Offset", offset);
        SmartDashboard.putNumber("Wheel Torque", 0);
        SmartDashboard.putNumber("Wheel Speed", 0);

        driveEnc = speedMotor.getEncoder();
        resetOdometry();

        angleMotor.setSmartCurrentLimit(50);
        speedMotor.setSmartCurrentLimit(50);
        angleMotor.setIdleMode(IdleMode.kBrake);
        speedMotor.setIdleMode(IdleMode.kBrake);

    }

    public void setPID(double p, double i, double d, double iScaling) {
        betterPID.setPID(p, i, d);
        betterPID.setiScaling(iScaling);
    }

    public void driveRaw(double speed, double goalAngle, boolean clip) {
        this.speed = speed;

        double processVariable = encoder.getPosition() - relativeOffset;
        double currentAngle = (processVariable * (360 / RobotConstants.swerveDriveGearRatio));
        double diff = (currentAngle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
            diff = diff - 360 * Math.signum(diff);
        }
        if (Math.abs(diff) > 90) {
            diff = diff - 180 * Math.signum(diff);
            speed *= -1;
        }

        double realGoalRotations = (currentAngle - diff) * RobotConstants.swerveDriveGearRatio / 360 + relativeOffset;

        //SmartDashboard.putNumber("Current Angle " + wheelName, currentAngle);
        //SmartDashboard.putNumber("Goal Angle " + wheelName, goalAngle);
        //SmartDashboard.putNumber("difference " + wheelName, diff);
        //SmartDashboard.putNumber("SPEED " + wheelName + wheelName, speed);
        //SmartDashboard.putNumber("Absolute Encoder Angle " + wheelName, (offset + getAbsoluteEncoderValue()) * (360 / RobotConstants.swerveDriveGearRatio));

        if (speed != 0 || diagnostic) {
            pidController.setReference(realGoalRotations, CANSparkMax.ControlType.kPosition);
        }

        betterPID.setTarget(RobotConstants.maxSwerveSpeed * speed); //3 casualties (1/25)
        double velocity = driveEncoder.getVelocity();

        DCMotor motor = DCMotor.getNEO(1);
        double v = motor.getVoltage(
            SmartDashboard.getNumber("Wheel Torque", 0),
            SmartDashboard.getNumber("Wheel Speed", 0)
        );
        SmartDashboard.putNumber("Wheel Applied Voltage", v);
        speedMotor.set(clip && Math.abs(speed) < .075 ? 0 : speed);
        SmartDashboard.putNumber(wheelName + " max speed", Math.max(SmartDashboard.getNumber(wheelName + " max speed", 0), Math.abs(speedMotor.get())));
    }

    public void drive(double speed, double goalAngle, boolean clip) {
        this.speed = speed;

        double processVariable = encoder.getPosition() - relativeOffset;
        double currentAngle = (processVariable * (360 / RobotConstants.swerveDriveGearRatio));
        double diff = (currentAngle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
            diff = diff - 360 * Math.signum(diff);
        }
        if (Math.abs(diff) > 90) {
            diff = diff - 180 * Math.signum(diff);
            speed *= -1;
        }

        double realGoalRotations = (currentAngle - diff) * RobotConstants.swerveDriveGearRatio / 360 + relativeOffset;

        //SmartDashboard.putNumber("Current Angle " + wheelName, currentAngle);
        //SmartDashboard.putNumber("Goal Angle " + wheelName, goalAngle);
        //SmartDashboard.putNumber("difference " + wheelName, diff);
        //SmartDashboard.putNumber("SPEED " + wheelName + wheelName, speed);
        //SmartDashboard.putNumber("Absolute Encoder Angle " + wheelName, (offset + getAbsoluteEncoderValue()) * (360 / RobotConstants.swerveDriveGearRatio));

        if (speed != 0 || diagnostic) {
            pidController.setReference(realGoalRotations, CANSparkMax.ControlType.kPosition);
        }

        betterPID.setTarget(RobotConstants.maxSwerveSpeed * speed); //3 casualties (1/25)
        double velocity = driveEncoder.getVelocity();

        DCMotor motor = DCMotor.getNEO(1);
        double v = motor.getVoltage(
            SmartDashboard.getNumber("Wheel Torque", 0),
            SmartDashboard.getNumber("Wheel Speed", 0)
        );
        SmartDashboard.putNumber("Wheel Applied Voltage", v);
        speedMotor.set(clip && Math.abs(speed) < .01 ? 0 : -betterPID.update(velocity));
        SmartDashboard.putNumber(wheelName + " max speed", Math.max(SmartDashboard.getNumber(wheelName + " max speed", 0), Math.abs(speedMotor.get())));
    }

    public void setEncoders(double offset) {
        offset = SmartDashboard.getNumber(wheelName + " Offset", offset);
        //SmartDashboard.putNumber(wheelName + " set pos rel", encoder.getPosition());
        double absolutePosition = offset + (absoluteEncoder.getSelectedSensorPosition(0) % 4096) * RobotConstants.swerveDriveGearRatio / 4096.0;

        //SmartDashboard.putNumber(wheelName + " set pos abs", absolutePosition);
        relativeOffset = encoder.getPosition() - absolutePosition;
        //SmartDashboard.putNumber(wheelName + "set pos offset", relativeOffset);
        //SmartDashboard.putNumber(wheelName + " set pos after", encoder.getPosition() + relativeOffset);
        this.drive(0.000000000001, 90, false);
    }

    public double getAbsoluteEncoderValue() {
        return ((absoluteEncoder.getSelectedSensorPosition(0) % 4096) * RobotConstants.swerveDriveGearRatio / 4096.0);
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();

    }


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            -(offset2 + driveEnc.getPosition() - (encoder.getPosition() - angleMarker) / ANGLE_TO_DRIVE_RATIO) / DRIVE_RATIO * 2 * Math.PI * WHEEL_RAD,
            Rotation2d.fromRotations(((encoder.getPosition() - relativeOffset) / RobotConstants.swerveDriveGearRatio))
        );
    }

    public void resetOdometry() {
        //driveEnc.setPosition(0);
        offset = -driveEnc.getPosition();
        angleMarker = encoder.getPosition();
    }

    public void displayPosition() {
        SwerveModulePosition position = getPosition();
        SmartDashboard.putNumber(wheelName + " Distance", position.distanceMeters);
        SmartDashboard.putNumber(wheelName + " Angle", position.angle.getRotations());
    }

    public void goofyDrive() {

        double speed = 1;

        double processVariable = encoder.getPosition() - relativeOffset;
        double currentAngle = (processVariable * (360 / RobotConstants.swerveDriveGearRatio));
        double diff = currentAngle % 360;

        if (Math.abs(diff) > 180) {
            diff = diff - 360 * Math.signum(diff);
        }
        if (Math.abs(diff) > 90) {
            diff = diff - 180 * Math.signum(diff);
            speed *= -1;
        }
        double realGoalRotations = (currentAngle - diff) * RobotConstants.swerveDriveGearRatio / 360 + relativeOffset;
        
        pidController.setReference(realGoalRotations, CANSparkMax.ControlType.kPosition);
        speedMotor.set(speed);

        // REMOVE SWERVE DRIVE PID

    }

}