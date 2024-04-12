package frc.robot;

import static frc.robot.RobotConstants.feederAssistMotorSpeed;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
     private final TalonFX intakeMotor;
     private final Slot0Configs config;
     private final MotionMagicVelocityVoltage request;
     private int stallCounter = 0;
     private final int stallThresh = 75;
     private boolean stalled = false;

     public Intake() {

          intakeMotor = new TalonFX(RobotConstants.intakeMotorID);
          intakeMotor.clearStickyFaults();
          intakeMotor.setNeutralMode(NeutralModeValue.Coast);

          config = new Slot0Configs();
          request = new MotionMagicVelocityVoltage(0);
          request.withSlot(0);

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

     public void displayConfig() {
          config
               .withKS(SmartDashboard.getNumber("Intake kS", 0))
               .withKV(SmartDashboard.getNumber("Intake kV", 0))
               .withKA(SmartDashboard.getNumber("Intake kA", 0))
               .withKP(SmartDashboard.getNumber("Intake kP", 0))
               .withKI(SmartDashboard.getNumber("Intake kI", 0))
               .withKD(SmartDashboard.getNumber("Intake kD", 0));
          intakeMotor.getConfigurator().apply(config);
          request
               .withSlot(0)
               .withVelocity(SmartDashboard.getNumber("Intake Vel", 0))
               .withAcceleration(SmartDashboard.getNumber("Intake Accel", 0));
     }

     public void displayDiagnostics() {
          SmartDashboard.putNumber("Intake Motor Temp", intakeMotor.getDeviceTemp().getValueAsDouble());
          SmartDashboard.putNumber("Intake Motor Torque Current", intakeMotor.getTorqueCurrent().getValueAsDouble());
          // Move to periodic
          if (Math.abs(intakeMotor.getTorqueCurrent().getValueAsDouble()) > stallThresh) stallCounter++;
          else stallCounter = 0;
          stalled = stalled || stallCounter > 16;
     }

     public boolean isStalled() {
          return stalled;
     }

     public boolean tempHigh() {
          return intakeMotor.getDeviceTemp().getValueAsDouble() > 80;
     }

     public void runGoofyIntake() {
          intakeMotor.setControl(request);
     }

     public void slurp() {
          if (stalled) stop();
          else intakeMotor.set(-RobotConstants.intakeMotorSpeed);
     }

     public void slurp(double speed) {
          if (stalled) stop();
          else intakeMotor.set(-speed);
     }

     public void fullPow() {
          intakeMotor.set(-1);
     }

     public void spit() {
          if (stalled) stop();
          else intakeMotor.set(RobotConstants.intakeMotorSpeed / 2);
     }

     public void stop() {
          intakeMotor.set(0);
     }

     public void resetStallState() {
          stalled = false;
     }

     public void assist() {
          if (stalled) stop();
          else intakeMotor.set(-feederAssistMotorSpeed);
     }
}
