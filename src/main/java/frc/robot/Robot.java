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
    double x,y,r;

    xAxis = controller.getSwerveX();
    yAxis = controller.getSwerveY();
    rAxis = controller.getSwerveR();

    SmartDashboard.putNumber("X Acceleration", navX.getWorldLinearAccelX());
    SmartDashboard.putNumber("Y Acceleration", navX.getWorldLinearAccelY());
    SmartDashboard.putNumber("Z Acceleration", navX.getWorldLinearAccelZ());
    SmartDashboard.putNumber("Total xy Acceleration", Math.sqrt(Math.pow(navX.getWorldLinearAccelX(), 2) + Math.pow(navX.getWorldLinearAccelY(), 2)));

    if (controller.resetNavXButton()) {
      swerveDrive.setEncoders();
      swerveDrive.resetNavX();
      navX.resetDisplacement();
    }
    x = -1 * xAxis * Math.abs(xAxis) * speed;
    y = yAxis * Math.abs(yAxis) * speed;
    r = rAxis * Math.abs(rAxis) * speed;

    swerveDrive.drive(x, y, r, true);

      if(controller.getButtonX()){
        swerveDrive.resetMaxVel();
      }
    
  }

  @Override
  public void autonomousInit() {
    swerveDrive.setEncoders();
  }

  @Override
  public void autonomousPeriodic() {


  }
}