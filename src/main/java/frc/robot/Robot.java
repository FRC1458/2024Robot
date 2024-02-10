package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class Robot extends TimedRobot {

  private final Controller controller;

  private final double speed;

  SwerveDrive swerveDrive;
  Pose2d robotPosition;

  Intake intake;
  Plumbing plumbing;
  Shooter shooter;

  private boolean intakeOn;


  private final AHRS navX;


  public Robot() {
    super(0.03);
    controller = new XboxController();

    navX = new AHRS(SPI.Port.kMXP);
    swerveDrive = new SwerveDrive(navX);
    intake = new Intake();
    plumbing = new Plumbing();
    shooter = new Shooter();

    speed = RobotConstants.speed;

  }

  @Override
  public void robotInit() {
    swerveDrive.resetNavX(new Pose2d(RobotConstants.initialXPos, RobotConstants.initialYPos, Rotation2d.fromDegrees(navX.getAngle())));
  }

  @Override
  public void robotPeriodic() {
    // robotPosition = swerveDrive.updateOdometry();
    //check and make sure it works
    SmartDashboard.putNumber("RobotXPos", robotPosition.getX());
    SmartDashboard.putNumber("RobotYPos", robotPosition.getY());
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

    //check acceleration for acceleration limiter? otherwise can delete next 4 lines
    SmartDashboard.putNumber("X Acceleration", navX.getWorldLinearAccelX());
    SmartDashboard.putNumber("Y Acceleration", navX.getWorldLinearAccelY());
    SmartDashboard.putNumber("Z Acceleration", navX.getWorldLinearAccelZ());
    SmartDashboard.putNumber("Total xy Acceleration", Math.sqrt(Math.pow(navX.getWorldLinearAccelX(), 2) + Math.pow(navX.getWorldLinearAccelY(), 2)));

    if (controller.resetNavXButton()) {
      swerveDrive.setEncoders();
      swerveDrive.resetNavX(robotPosition);
      navX.resetDisplacement();
    }
    x = -1 * xAxis * Math.abs(xAxis) * speed;
    y = yAxis * Math.abs(yAxis) * speed;
    r = rAxis * Math.abs(rAxis) * speed;

    swerveDrive.drive(x, y, r, true);

    if(controller.getButtonX()){
      swerveDrive.resetMaxVel();
    }

    if (controller.getButtonAPressed()){ //toggle intake on/off
      intakeOn = !intakeOn;
      if (intakeOn) {
        intake.slurp();
      }
      else {
        intake.stop();
      }
    }

    if(controller.getLeftTrigger()){ //rev up shooter motors, to be changed
      shooter.shoot();
    }
    else{
      shooter.stop();
    }

    if(controller.getRightTrigger()){ //"shoot" the piece into the spinning shooter
      plumbing.eject();
    }
    else{
      plumbing.clog();
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