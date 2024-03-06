package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;

public class Robot extends TimedRobot {
  

  private final XboxController xbox;

  private final IFS ifs;

  SwerveDrive swerveDrive;
  Pose2d robotPosition;

  Intake intake;
  Feeder feeder;
  Shooter shooter;

  private final AHRS navX;

  StateMachine<BasicAuto.AutoStates> auto;

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;


  public Robot() {
    super(0.02);
    xbox = new XboxController(0);

    navX = new AHRS(SPI.Port.kMXP);
    swerveDrive = new SwerveDrive(navX);
    intake = new Intake();
    feeder = new Feeder();
    shooter = new Shooter();

    //ifs = new IFSManual(intake, feeder, shooter, xbox);
    ifs = new IFSAuto(intake, feeder, shooter, xbox);

  }

  @Override
  public void robotInit() {
    swerveDrive.resetNavX(new Pose2d(RobotConstants.initialXPos, RobotConstants.initialYPos, Rotation2d.fromDegrees(navX.getAngle())));
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void robotPeriodic() {
    // robotPosition = swerveDrive.updateOdometry();
    //check and make sure it works
    // SmartDashboard.putNumber("RobotXPos", robotPosition.getX());
    // SmartDashboard.putNumber("RobotYPos", robotPosition.getY());
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

    xAxis = xbox.getLeftX();
    yAxis = xbox.getLeftY();
    rAxis = xbox.getRightX();

    //check acceleration for acceleration limiter? otherwise can delete next 4 lines
    SmartDashboard.putNumber("X Acceleration", navX.getWorldLinearAccelX());
    SmartDashboard.putNumber("Y Acceleration", navX.getWorldLinearAccelY());
    SmartDashboard.putNumber("Z Acceleration", navX.getWorldLinearAccelZ());
    SmartDashboard.putNumber("Total xy Acceleration", Math.sqrt(Math.pow(navX.getWorldLinearAccelX(), 2) + Math.pow(navX.getWorldLinearAccelY(), 2)));

    if (xbox.getStartButton()) {
      swerveDrive.setEncoders();
      swerveDrive.resetNavX(robotPosition);
      navX.resetDisplacement();
    }
    x = -xAxis * Math.abs(xAxis);
    y = yAxis * Math.abs(yAxis);
    r = rAxis * Math.abs(rAxis);

    swerveDrive.drive(x, y, r, true);

    if(xbox.getXButton()){
      swerveDrive.resetMaxVel();
      shooter.stopPivot();
    }

    ifs.update();
  }

  @Override
  public void autonomousInit() {
    swerveDrive.setEncoders();
    auto = BasicAuto.getStateMachine(feeder, shooter, swerveDrive);
    auto.reset();
  }

  @Override
  public void autonomousPeriodic() {
    auto.run();
  }

  @Override
  public void testInit() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
    led.setData(ledBuffer);
  }

}