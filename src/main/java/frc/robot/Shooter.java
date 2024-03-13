package frc.robot;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static frc.robot.RobotConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.FixedPivot;
import frc.robot.motors.Pivot;
import frc.robot.swervedrive.PID;


public class Shooter {
   private final TalonFX rightShooter;
   private final TalonFX leftShooter;
   private final Pivot pivot;

   private final PID rightPID;
   private final PID leftPID;
   
     public Shooter() {

          rightShooter = new TalonFX(rightShooterMotorID);
          leftShooter = new TalonFX(leftShooterMotorID);
          pivot = new FixedPivot(-4.3);

          rightShooter.clearStickyFaults();
          leftShooter.clearStickyFaults();
          
          rightShooter.setNeutralMode(Coast);
          leftShooter.setNeutralMode(Coast);
          
          rightPID = new PID();
          leftPID = new PID();
          configurePID();

          SmartDashboard.putNumber("AMP Pivot Position", -4.3);
          SmartDashboard.putNumber("AMP Shooter Speed", .14);
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
          SmartDashboard.putNumber("Right Shooter RPS", rightShooter.getVelocity().getValueAsDouble());
          SmartDashboard.putNumber("Left Shooter RPS", leftShooter.getVelocity().getValueAsDouble());
         updatePID();

         double pl = leftPID.update(leftShooter.getVelocity().getValueAsDouble(), speed * MAX_SHOOTER_RPS) / MAX_SHOOTER_RPS;
         SmartDashboard.putNumber("Percentage Left", pl);
          leftShooter.set(-pl);

          double pr = rightPID.update(rightShooter.getVelocity().getValueAsDouble(), speed * MAX_SHOOTER_RPS) / MAX_SHOOTER_RPS;
         SmartDashboard.putNumber("Percentage Right", pr);
          rightShooter.set(pr);
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

     public void shootAmp() {
         setSpeed(SmartDashboard.getNumber("AMP Shooter Speed", .14));
     }

     public void updatePID() {
         leftPID.updatePID("Left Shooter");
         rightPID.updatePID("Right Shooter");
     }

     public void displayPivot() {
         SmartDashboard.putNumber("Pivot Position", pivot.getPosition());
     }

}