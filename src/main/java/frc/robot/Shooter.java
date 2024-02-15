package frc.robot;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static frc.robot.RobotConstants.*;
import static frc.robot.util.MotorUtil.targetSpeed;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.PID;

public class Shooter {
   private final TalonFX rightShooter;
   private final TalonFX leftShooter;

   private final SlewRateLimiter rightFilter;
   private final SlewRateLimiter leftFilter;

   private final PID rightPID;
   private final PID leftPID;
   
     public Shooter() {
          rightShooter = new TalonFX(rightShooterMotorID);
          leftShooter = new TalonFX(leftShooterMotorID);
          rightShooter.clearStickyFaults();
          leftShooter.clearStickyFaults();
          rightShooter.setNeutralMode(Coast);
          leftShooter.setNeutralMode(Coast);
          rightFilter = new SlewRateLimiter(shooterSpeed/shooterRampUpTime);
          leftFilter = new SlewRateLimiter(shooterSpeed/shooterRampUpTime);
          rightPID = new PID();
          leftPID = new PID();
          configurePID();
     }

     public void configurePID() {
          TalonFXConfiguration talonConfig = new TalonFXConfiguration();
          talonConfig.MotionMagic.MotionMagicAcceleration = 160;
          talonConfig.MotionMagic.MotionMagicJerk = 1600;

          talonConfig.Slot0.kP = 1;
          talonConfig.Slot0.kI = 0;
          talonConfig.Slot0.kD = 0.1;

          rightShooter.getConfigurator().apply(talonConfig);
          leftShooter.getConfigurator().apply(talonConfig);

          double p = 0.025;
          rightPID.setPID(p, 0, 0);
          leftPID.setPID(p, 0, 0);

     }

     public void shoot() {
          rightShooter.set(rightFilter.calculate(shooterSpeed * -1));
          leftShooter.set(leftFilter.calculate(shooterSpeed));
     }

     public void stop() {
          rightShooter.set(0);
          leftShooter.set(0);
          // rightShooter.setControl(targetSpeed(0));
          // leftShooter.setControl(targetSpeed(0));

     }

     public void scoreSpeakerPID() {
          // rightShooter.setControl(targetSpeed(shooterPIDSpeed));
          // leftShooter.setControl(targetSpeed(-shooterPIDSpeed));
          SmartDashboard.putNumber("Right Shooter RPM", rightShooter.getVelocity().getValueAsDouble());
          //SmartDashboard.putNumber("Left Shooter RPM", leftShooter.getVelocity().getValueAsDouble());
          SmartDashboard.putNumber("Right Shooter Voltage", rightPID.update(rightShooter.getVelocity().getValue(), shooterPIDSpeed));
          //SmartDashboard.putNumber("Left Shooter Voltage", leftPID.update(leftShooter.getVelocity().getValue(), shooterPIDSpeed));
          rightShooter.set(0.9);//rightPID.update(rightShooter.getVelocity().getValue(), shooterPIDSpeed));
          leftShooter.set(-0.9);
     }
}