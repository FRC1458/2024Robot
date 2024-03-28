package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.wrappers.JoystickWrapper;
import frc.robot.wrappers.TalonFXWrapper;
import frc.robot.wrappers.XboxControllerWrapper;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static frc.robot.RobotConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.*;



public class Robot extends TimedRobot {
  private TalonFX motor;
  private TalonFXConfiguration motorConfig;

  public Robot() {
    super(0.03);

    motor = new TalonFX(51);  
    motorConfig = new TalonFXConfiguration();

  }
  @Override
  public void robotInit() {
  }
  @Override
  public void teleopInit() {
    motorConfig.Slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    motorConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    motorConfig.Slot0.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    motorConfig.Slot0.kI = 0; // no output for integrated error
    motorConfig.Slot0.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0; //80;
    motorConfig.MotionMagic.MotionMagicAcceleration = 160;
    motorConfig.MotionMagic.MotionMagicJerk = 1600;
    motor.getConfigurator().apply(motorConfig);
  }
  @Override
  public void teleopPeriodic() {
    motor.set(SmartDashboard.getNumber("motor speed (percent)", 0.25));
  }

  @Override
  public void autonomousInit() {
  }
  @Override
  public void autonomousPeriodic() {
  }
  @Override public void testInit() {
  }
  @Override public void testPeriodic() {
  }
}


