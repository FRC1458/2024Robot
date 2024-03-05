package frc.robot.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import static frc.robot.RobotConstants.*;

public class Pivot {
    
    private static final double MAX_SPEED = -0.035;
    private Double holdingPosition = null;
    private double kP = 0.00000001;
    private final TalonFX pivot = new TalonFX(RobotConstants.pivotMotorID);
   private final DigitalInput resetSwitch = new DigitalInput(pivotLimSwitchChannel);
    private boolean pivotToggle = false;

    public Pivot() {
        pivot.clearStickyFaults();
        pivot.setNeutralMode(NeutralModeValue.Brake);
        pivotToggle = resetSwitch.get();
    }

    public void setSpeed(double speed) {

        holdingPosition = null;
        if (speed < 0 && resetSwitch.get()) {
            setSpeed(0);
            pivotToggle = true;
        } else set(speed);

        if (!resetSwitch.get() && pivotToggle) {
            pivot.setPosition(0);
            pivotToggle = false;
        }

        SmartDashboard.putNumber("Pivot Position", pivot.getPosition().getValueAsDouble());
    }

    private void set(double speed) {
        if (Math.abs(speed) > 0.035) pivot.stopMotor();
        else pivot.set(Math.signum(speed) * Math.min(Math.abs(speed), MAX_SPEED));
    }

    public void hold() {
        double position = getPosition();
        if (holdingPosition == null) holdingPosition = position;
        double err = position - holdingPosition;
        if (err <= 0) {
            stop();
            return;
        }
        set(kP * err);
    }

    public double getPosition() {
        return pivot.getPosition().getValueAsDouble();
    }

    public void stop() {
        pivot.stopMotor();
    }

}