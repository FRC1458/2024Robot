package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.wrappers.TalonFXWrapper;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Arm {

    private final Solenoid armSolenoid;
    private final TalonFXWrapper armMotor;
    private armStates state = armStates.IDLE;
    private boolean armExtended = false;

    private enum armStates {
        TOP,
        MIDDLE,
        BOTTOM,
        IDLE
    }

    public enum intakeStates {
        ACCEPT,
        REJECT,
        STOP
    }
    public intakeStates intakeState = intakeStates.STOP;
    
    private final static int clawID = 8;
    private final static double clawSpeed = .35;
    public final static CANSparkMax claw = new CANSparkMax(clawID, MotorType.kBrushless);

    public Arm(Solenoid armSolenoid) {
        
        claw.clearFaults();
        claw.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        
        this.armSolenoid = armSolenoid;
        armMotor = new TalonFXWrapper(43, true);
        retractArm();
    }
    
    public double encoderPosition() {
        TalonFXSensorCollection sensors = armMotor.talon.getSensorCollection();
        double absPos = sensors.getIntegratedSensorPosition() * RobotConstants.armEncoderRatio;
        //System.out.print("Encoder: " + absPos);
        SmartDashboard.putNumber("Arm encoder: ", absPos);
        return absPos;
    }

    public void reset() {
        TalonFXSensorCollection sensors = armMotor.talon.getSensorCollection();
        double new_position = 5 / RobotConstants.armEncoderRatio;
        sensors.setIntegratedSensorPosition(new_position, 0);
    }

    public void runArm(boolean goDown, boolean goUp) {
        if (goDown || goUp) {
            state = armStates.IDLE;
        }

        switch (state) {
            case TOP:
                up();
                break;
            case MIDDLE:
                middle();
                break;
            case BOTTOM:
                down();
                break;
            case IDLE:
                runManual(goDown, goUp);
                break;
        }
    }

    public void clawAccept() {if (intakeState != intakeStates.ACCEPT) claw.set(clawSpeed); intakeState = intakeStates.ACCEPT;}
    public void clawStop() {if (intakeState != intakeStates.STOP) claw.stopMotor(); intakeState = intakeStates.STOP;}
    public void clawReject() {if (intakeState != intakeStates.REJECT) claw.set(-clawSpeed); intakeState = intakeStates.REJECT;}

    public void setUp() {
        state = armStates.TOP;
    }

    public void setMiddle() {
        state = armStates.MIDDLE;
    }
    
    public void setBottom() {
        state = armStates.BOTTOM;
    }

    public void runManual(boolean goDown, boolean goUp) {
        if (goDown && (encoderPosition() > 42 || !armExtended)) {
            moveDown(RobotConstants.armSpeed);
        } else if (goUp) {
            moveUp(RobotConstants.armSpeed);
        } else {
            armMotor.set(0);
        }
    }


    public void up() {
        moveToPreset(105);
    }

    public void middle() {
        moveToPreset(90);
    }

    public void down() {
        moveToPreset(45);
        if (!armExtended) {
            extendArm();
        }
    }

    public void moveToPreset(double presetAngle) {
        if (encoderPosition() < presetAngle - 1.5) {
            if (encoderPosition() < presetAngle - 10) {
                moveUp(RobotConstants.armSpeed + .1);
            } else {
                moveUp(RobotConstants.armSpeed);
            }
        } else if (encoderPosition() > presetAngle + 1.5) {
            if (encoderPosition() > presetAngle + 10) {
                moveDown(RobotConstants.armSpeed + .1);
            } else {
                moveDown(RobotConstants.armSpeed);
            }
        }
        else {
            state = armStates.IDLE;
        }      
    }

    public void moveUp(double speed) {
        if (encoderPosition() < 120) {
            armMotor.set(speed);
        } else {
            armMotor.set(0);
        }
    }

    public void moveDown(double speed) {
        armMotor.set(-1 * speed);
    }

    public void extendArm() {
        if (encoderPosition() > 42) {
            armExtended = true;
            armSolenoid.forward();
        }
    }

    public void retractArm() {
        armExtended = false;
        armSolenoid.reverse();
    }

}