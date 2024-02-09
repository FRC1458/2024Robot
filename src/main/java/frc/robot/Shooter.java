package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Shooter {
   private final TalonFX rightShooter;
   private final TalonFX leftShooter;

   private final SlewRateLimiter rightFilter;
   private final SlewRateLimiter leftFilter;
   
     public Shooter() {
          rightShooter = new TalonFX(RobotConstants.rightShooterMotorID);
          leftShooter = new TalonFX(RobotConstants.leftShooterMotorID);
          rightShooter.clearStickyFaults();
          leftShooter.clearStickyFaults();
          rightShooter.setNeutralMode(NeutralModeValue.Coast);
          leftShooter.setNeutralMode(NeutralModeValue.Coast);
          rightFilter = new SlewRateLimiter(RobotConstants.shooterSpeed/RobotConstants.shooterRampUpTime);
          leftFilter = new SlewRateLimiter(RobotConstants.shooterSpeed/RobotConstants.shooterRampUpTime);
     }

     public void shoot() {
          rightShooter.set(rightFilter.calculate(RobotConstants.shooterSpeed * -1));
          leftShooter.set(leftFilter.calculate(RobotConstants.shooterSpeed));
     }

     public void stop() {
          rightShooter.set(0);
          leftShooter.set(0);
     }
}