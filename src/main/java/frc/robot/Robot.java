package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class Robot extends TimedRobot {

  private final Controller controller;

  private final double speed;

  SwerveDrive swerveDrive;


  private final AHRS navX;


  public Robot() {
    super(0.03);
    controller = new XboxController();

    navX = new AHRS(SPI.Port.kMXP);
    swerveDrive = new SwerveDrive(navX);

    speed = RobotConstants.speed;

    SmartDashboard.putNumber("Robot Speed", speed);

  }

  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
    swerveDrive.setEncoders();
  }

  @Override
  public void teleopPeriodic() {
    double xAxis;
    double yAxis;
    double rAxis;
    double x,y,r,speedIncrease;
    speedIncrease = SmartDashboard.getNumber("Robot Speed", speed);

    xAxis = controller.getSwerveX();
    yAxis = controller.getSwerveY();
    rAxis = controller.getSwerveR();



    if (controller.resetNavX()) {
      swerveDrive.setEncoders();
      swerveDrive.resetNavX();
      navX.resetDisplacement();
    }

    x = -xAxis*Math.abs(xAxis) * speedIncrease;
    y = yAxis*Math.abs(yAxis)* speedIncrease;
    r = rAxis*Math.abs(rAxis) * speedIncrease;

    swerveDrive.drive(x, y, r, true);

  }

  @Override
  public void autonomousInit() {
    swerveDrive.setEncoders();
  }

  @Override
  public void autonomousPeriodic() {


  }
}