package frc.robot;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static frc.robot.RobotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.Pivot;
import frc.robot.swervedrive.PID;


public class Shooter {
   private final TalonFX rightShooter;
   private final TalonFX leftShooter;
   private final Pivot pivot;

   private TalonFXConfiguration rightConfig, leftConfig;

    private double ampSpeed = 0.14;
   

   private final PID rightPID;
   private final PID leftPID;
   
     public Shooter() {
          rightShooter = new TalonFX(rightShooterMotorID);
          leftShooter = new TalonFX(leftShooterMotorID);
          rightConfig = new TalonFXConfiguration();
          leftConfig = new TalonFXConfiguration();

          Slot0Configs slot0configs = new Slot0Configs();
          slot0configs.kS = 0.01;
          slot0configs.kV = 0.11;
          slot0configs.kP = 0.1;
          slot0configs.kI = 0;
          slot0configs.kD = 0;

          rightShooter.getConfigurator().apply(slot0configs);
          leftShooter.getConfigurator().apply(slot0configs);

          pivot = new Pivot();

          rightShooter.clearStickyFaults();
          leftShooter.clearStickyFaults();
          
          rightShooter.setNeutralMode(Coast);
          leftShooter.setNeutralMode(Coast);

          //Make sure foc is disabled on shooter motors

          
          rightPID = new PID();
          leftPID = new PID();
          configurePID();

          SmartDashboard.putNumber("AMP Pivot Position", -4.3);
          SmartDashboard.putNumber("AMP Shooter Speed", ampSpeed);
          SmartDashboard.putNumber("Pivot Test Angle", -4.3);

     }

     public void configurePID() {

          rightPID.setPID(0.50, 0, 0);
          leftPID.setPID(3.25, 0.15, 0.45);
         leftPID.initDebug("Left Shooter");
         rightPID.initDebug("Right Shooter");

          rightPID.setMaxAccel(180);
          leftPID.setMaxAccel(180);

     }

     public void stop() {
          rightShooter.set(0);
          leftShooter.set(0);
         rightPID.reset();
         leftPID.reset();

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
          //SmartDashboard.putNumber("Left Shooter RPS", speed * MAX_SHOOTER_RPS / MAX_SHOOTER_RPS);
          //SmartDashboard.putNumber("Left Shooter RPS", speed * MAX_SHOOTER_RPS / MAX_SHOOTER_RPS);
         //updatePID();

          //double pl = leftPID.update(leftShooter.getVelocity().getValueAsDouble(), speed * MAX_SHOOTER_RPS) / MAX_SHOOTER_RPS;
          //SmartDashboard.putNumber("Percentage Left", pl);
          //leftShooter.set(-pl);
          //var ts = speed * MAX_SHOOTER_RPS / MAX_SHOOTER_RPS;
          //leftConfig.MotionMagic.MotionMagicCruiseVelocity = -ts;
          //leftConfig.MotionMagic.MotionMagicAcceleration = -ts * 2;
          //leftConfig.MotionMagic.MotionMagicJerk = 1600;
          //leftShooter.getConfigurator().apply(leftConfig);
          //leftShooter.set(leftConfig.MotionMagic.MotionMagicCruiseVelocity);

          //double pr = rightPID.update(rightShooter.getVelocity().getValueAsDouble(), speed * MAX_SHOOTER_RPS) / MAX_SHOOTER_RPS;
          //SmartDashboard.putNumber("Percentage Right", pr);
          //rightShooter.set(pr);

          //rightConfig.MotionMagic.MotionMagicCruiseVelocity = ts;
          //rightConfig.MotionMagic.MotionMagicAcceleration = ts * 2;
          //rightConfig.MotionMagic.MotionMagicJerk = 1600;
          //rightShooter.getConfigurator().apply(rightConfig);
          //rightShooter.set(rightConfig.MotionMagic.MotionMagicCruiseVelocity);
         double rps = speed * MAX_SHOOTER_RPS;
         rightShooter.setControl(new VelocityVoltage(0).withSlot(0).withVelocity(-rps).withFeedForward(0));
         leftShooter.setControl(new VelocityVoltage(0).withSlot(0).withVelocity(rps).withFeedForward(0));
         SmartDashboard.putNumber("left shooter rps", leftShooter.getRotorVelocity().getValue());
         SmartDashboard.putNumber("right shooter rps", rightShooter.getRotorVelocity().getValue());
     }

     private boolean pivotTo(double desiredPosition) {
        double position = pivot.getPosition();
        SmartDashboard.putNumber("Desired Position", desiredPosition);
        SmartDashboard.putNumber("Position", position);
        SmartDashboard.putBoolean("Move Up?", position > desiredPosition + 0.1);
        SmartDashboard.putBoolean("Move Down?", position < desiredPosition - 0.1);

        if (position > desiredPosition + 0.25) moveUp();
        else if (position < desiredPosition - 0.25) moveDown();
        else {
            stopPivot();
            return true;
        }
        return false;
     }

     public boolean pivotToTest() {
        return pivotTo(SmartDashboard.getNumber("Pivot Test Angle", -4.3));
     }

    public boolean pivotToSpeaker() {
        return pivotTo(-1.8);
    }
    public boolean pivotPointBlank() {
         return pivotTo(-4.3);
    }

    public boolean pivotToAmp() {
        return pivotTo(SmartDashboard.getNumber("AMP Pivot Position", -4.3));
    }

     public void shootSpeaker() {
        
        setSpeed(shooterSpeedSpeaker);
     }


     //public void increaseAmpSpeed() {
       // ampSpeed += 0.01;
     //}

     //public void decreaseAmpSpeed() {
       // ampSpeed -= 0.01;
     //}

     public void shootAmp() {
         setSpeed(ampSpeed);
     }

     public void updatePID() {
         leftPID.updatePID("Left Shooter");
         rightPID.updatePID("Right Shooter");
     }

     public void displayPivot() {
         SmartDashboard.putNumber("Pivot Position", pivot.getPosition());
     }

    public double getAmpSpeed() {
        return ampSpeed;
    }

    public void reverse() {
        leftShooter.set(-0.5);
        rightShooter.set(0.5);
    }

}