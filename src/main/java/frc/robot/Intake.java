package frc.robot;

import static frc.robot.RobotConstants.feederAssistMotorSpeed;
import static frc.robot.RobotConstants.intakeMotorSpeed;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
     private final TalonFX intakeMotor1;
     private final TalonFX intakeMotor2;
     private final Slot0Configs config;
     private int stallCounter = 0;
     private final int stallThresh = 75;
     private boolean stalled = false;

     public Intake() {

          intakeMotor1 = new TalonFX(RobotConstants.intakeMotorID);
          intakeMotor1.clearStickyFaults();
          intakeMotor1.setNeutralMode(NeutralModeValue.Coast);

          

          intakeMotor2 = new TalonFX(36);
          intakeMotor2.clearStickyFaults();
          intakeMotor2.setNeutralMode(NeutralModeValue.Coast);

          config = new Slot0Configs();

          // initConfig();

     }

     public void initConfig() {
          SmartDashboard.putNumber("Intake kS", 0);
          SmartDashboard.putNumber("Intake kV", 0);
          SmartDashboard.putNumber("Intake kA", 0);
          SmartDashboard.putNumber("Intake kP", 0);
          SmartDashboard.putNumber("Intake kI", 0);
          SmartDashboard.putNumber("Intake kD", 0);
          SmartDashboard.putNumber("Intake Vel", 0);
          SmartDashboard.putNumber("Intake Accel", 0);
     }



     public void displayDiagnostics() {
          SmartDashboard.putNumber("Intake Motor Temp", intakeMotor1.getDeviceTemp().getValueAsDouble());
          SmartDashboard.putNumber("Intake Motor Torque Current", intakeMotor1.getTorqueCurrent().getValueAsDouble());
          // Move to periodic
          if (Math.abs(intakeMotor1.getTorqueCurrent().getValueAsDouble()) > stallThresh) stallCounter++;
          else stallCounter = 0;
          stalled = stalled || stallCounter > 16;
     }

     public boolean isStalled() {
          return stalled;
     }

     public boolean tempHigh() {
          return intakeMotor1.getDeviceTemp().getValueAsDouble() > 80;
     }



     public void slurp() {
          if (stalled) stop();
          else {
               setSpeed(intakeMotorSpeed);
          }
     }


     public void fullPow() {
          //intakeMotor1.set(-1);
     }

     public void spit() {
          if (stalled) stop();
          else {
               setSpeed(-intakeMotorSpeed);
          }
     }

     public void stop() {
          setSpeed(0);
     }

     public void resetStallState() {
          stalled = false;
     }

     public void assist() {
          if (stalled) stop();
          else {
               setSpeed(intakeMotorSpeed);
          }
     }

     private void setSpeed(double speed) {
          //DO NOT TOUCH THIS GOLLY GEE
          intakeMotor2.set(speed);
          intakeMotor1.set(-speed);
     }
}
