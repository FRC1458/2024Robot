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

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;



public class Robot extends TimedRobot {
  private TalonFX motor;
  private TalonFXConfiguration motorConfig;

  Slot0Configs slot0configs = new Slot0Configs();

  double ks = 0.01;
  double kv = 0.11;
  //double ka = 0.01;
  double kp = .10;
  double ki = 0;
  double kd = 0;


  public Robot() {
    super(0.03);

    motor = new TalonFX(50);  
    motorConfig = new TalonFXConfiguration();

  }
  @Override
  public void robotInit() {
  }
  @Override
  public void teleopInit() {
    slot0configs.kS = ks; // Add 0.25 V output to overcome static friction
    slot0configs.kV = kv; // A velocity target of 1 rps results in 0.12 V output
    //slot0configs.kA = ka; // An acceleration of 1 rps/s requires 0.01 V output
    slot0configs.kP = kp; // A position error of 2.5 rotations results in 12 V output
    slot0configs.kI = ki; // no output for integrated error
    slot0configs.kD = kd; // A velocity error of 1 rps results in 0.1 V output
    //motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0; //80;
    //motorConfig.MotionMagic.MotionMagicAcceleration = 160;
    //motorConfig.MotionMagic.MotionMagicJerk = 1600;
    motor.getConfigurator().apply(slot0configs);
    SmartDashboard.putNumber("motor speed (percent)", 0.25);
    SmartDashboard.putNumber("kS", ks);
    SmartDashboard.putNumber("kV", kv);
    //SmartDashboard.putNumber("kA", ka);
    SmartDashboard.putNumber("kP", kp);
    SmartDashboard.putNumber("kI", ki);
    SmartDashboard.putNumber("kD", kd);
    SmartDashboard.putNumber("MMCruiseVelocity", 0);
    SmartDashboard.putNumber("MMAcceleration", 160);
    SmartDashboard.putNumber("MMJerk", 1600);
    SmartDashboard.putNumber("motor speed (rps)", 0);
  }
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Rotor Velocity", (double) motor.getRotorVelocity().getValue());
    slot0configs.kS = SmartDashboard.getNumber("kS", ks); // Add 0.25 V output to overcome static friction
    slot0configs.kV = SmartDashboard.getNumber("kV", kv); // A velocity target of 1 rps results in 0.12 V output
    //slot0configs.kA = SmartDashboard.getNumber("kA", ka); // An acceleration of 1 rps/s requires 0.01 V output
    slot0configs.kP = SmartDashboard.getNumber("kP", kp); // A position error of 2.5 rotations results in 12 V output
    slot0configs.kI = SmartDashboard.getNumber("kI", ki); // no output for integrated error
    slot0configs.kD = SmartDashboard.getNumber("kD", kd);
    //motorConfig.MotionMagic.MotionMagicCruiseVelocity = SmartDashboard.getNumber("MMCruiseVelocity", 0);
    //motorConfig.MotionMagic.MotionMagicAcceleration = SmartDashboard.getNumber("MMAcceleration", 160);
    //motorConfig.MotionMagic.MotionMagicJerk = SmartDashboard.getNumber("MMJerk", 1600);
    motor.getConfigurator().apply(slot0configs);
    
    motor.setControl(new VelocityVoltage(0).withSlot(0).withVelocity(SmartDashboard.getNumber("motor speed (rps)", 0)).withFeedForward(0));
    //motor.set(SmartDashboard.getNumber("motor speed (percent)", 0.25));
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


