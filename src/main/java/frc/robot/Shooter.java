package frc.robot;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static frc.robot.RobotConstants.*;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.Pivot;

public class Shooter {

    private final TalonFX rightShooter;
    private final TalonFX leftShooter;
    private final Pivot pivot;

    private double ampSpeed = 0.12;

    public Shooter() {

        rightShooter = new TalonFX(rightShooterMotorID);
        leftShooter = new TalonFX(leftShooterMotorID);

        Slot0Configs slot0configs = new Slot0Configs();
        slot0configs.kS = 0.01;
        slot0configs.kV = 0.11;
        slot0configs.kP = 0.1;
        slot0configs.kI = 0;
        slot0configs.kD = 0;
        rightShooter.getConfigurator().apply(slot0configs);
        leftShooter.getConfigurator().apply(slot0configs);

        // SmartDashboard.setPersistent("Shooter KS");
        // SmartDashboard.containsKey("Shooter KS");
        // SmartDashboard.clearPersistent("Shooter KS");
        SmartDashboard.putNumber("Shooter kS", 0.125);
        SmartDashboard.putNumber("Shooter kV", 2.5);
        SmartDashboard.putNumber("Shooter kA", 0);
        SmartDashboard.putNumber("Shooter kP", 0);
        SmartDashboard.putNumber("Shooter kI", 0);
        SmartDashboard.putNumber("Shooter kD", 0);
        SmartDashboard.putNumber("Shooter Vel", 40);
        SmartDashboard.putNumber("Shooter Accel", 40);

        pivot = new Pivot();

        rightShooter.clearStickyFaults();
        leftShooter.clearStickyFaults();

        rightShooter.setNeutralMode(Coast);
        leftShooter.setNeutralMode(Coast);

        // Make sure foc is disabled on shooter motors

        SmartDashboard.putNumber("AMP Shooter Speed", ampSpeed);

    }

    public void stop() {
        rightShooter.set(0);
        leftShooter.set(0);

    }

    public void setPivotSpeed(double speed) {
        pivot.setSpeed(Math.signum(speed) * Math.min(Math.abs(speed), 0.05));
    }

    public void moveUp() {
        setPivotSpeed(MAX_PIVOT_SPEED);
    }

    public void moveDown() {
        setPivotSpeed(-PIVOT_DOWN_SPEED);
    }

    public void stopPivot() {
        pivot.hold();
    }

    private void setSpeed(double speed) {
        double rps = speed * MAX_SHOOTER_RPS;
        rightShooter.setControl(new VelocityVoltage(0).withSlot(0).withVelocity(-rps));
        leftShooter.setControl(new VelocityVoltage(0).withSlot(0).withVelocity(rps));
        SmartDashboard.putNumber("left shooter rps", leftShooter.getRotorVelocity().getValue());
        SmartDashboard.putNumber("right shooter rps", rightShooter.getRotorVelocity().getValue());
    }

    public void setSpeedRamp(double rps) {
        Slot1Configs rampUpConfigs = new Slot1Configs();
        rampUpConfigs
            .withKS(SmartDashboard.getNumber("Shooter kS", 0))
            .withKV(SmartDashboard.getNumber("Shooter kV", 0))
            .withKA(SmartDashboard.getNumber("Shooter kA", 0))
            .withKP(SmartDashboard.getNumber("Shooter kP", 0))
            .withKI(SmartDashboard.getNumber("Shooter kI", 0))
            .withKD(SmartDashboard.getNumber("Shooter kD", 0));
        rightShooter.getConfigurator().apply(rampUpConfigs);
        leftShooter.getConfigurator().apply(rampUpConfigs);
        rightShooter.setControl(new MotionMagicVelocityVoltage(-rps).withSlot(1).withAcceleration(SmartDashboard.getNumber("Shooter Accel", 0)));
        leftShooter.setControl(new MotionMagicVelocityVoltage(rps).withSlot(1).withAcceleration(SmartDashboard.getNumber("Shooter Accel", 0)));
    }

    public double shooterRPM() {
        return (Math.abs(leftShooter.getVelocity().getValueAsDouble())
                + Math.abs(rightShooter.getVelocity().getValueAsDouble())) / 2;
    }

    public boolean shooterRampedUp() {
        return shooterRPM() > 90;
    }

    public void shootSpeaker() {

        setSpeed(shooterSpeedSpeaker);
    }

    public void shootMax() {
        setSpeed(1);
    }

    public void increaseAmpSpeed() {
        ampSpeed += 0.01;
    }

    public void decreaseAmpSpeed() {
        ampSpeed -= 0.01;
    }

    public void shootAmp() {
        setSpeed(ampSpeed);
    }

    public double getAmpSpeed() {
        return ampSpeed;
    }

    public void reverse() {
        leftShooter.set(-0.5);
        rightShooter.set(0.5);
    }

    public void debug() {
        setSpeedRamp(SmartDashboard.getNumber("Shooter Vel", 0));
    }

}