package frc.robot.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.RobotConstants;

public class Pivot {
    
    private static final double MAX_SPEED = -0.035;
    TalonFX pivot = new TalonFX(RobotConstants.pivotMotorID);

    public Pivot() {
        pivot.clearStickyFaults();
        pivot.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speed) {
        if (Math.abs(speed) > 0.035) pivot.stopMotor();
        else pivot.set(Math.signum(speed) * Math.min(Math.abs(speed), MAX_SPEED));
    }

    public void stop() {
        pivot.stopMotor();
    }

}