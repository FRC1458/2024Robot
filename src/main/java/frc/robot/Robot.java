package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Trajectory.PathPlannerTraj;
import frc.robot.Trajectory.Trajectory;
import frc.robot.Trajectory.WPITraj;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.RobotConstants.*;


public class Robot extends TimedRobot {

  private final XboxController xbox;

  private final IFS ifs;

  public static DigitalInput irBreak;
  int count;
  Timer timer = new Timer();

  SwerveDrive swerveDrive;
  Pose2d robotPosition;
  Ultrasonic rangeFinder;

  Intake intake;
  Feeder feeder;
  Shooter shooter;


  private final AHRS navX;
  StateMachine<BasicAuto.AutoStates> auto;

  Trajectory trajectory;

  private LED lights;


  public Robot() {
    super(0.02);
    xbox = new XboxController(0);
    count = 0;

    navX = new AHRS(SPI.Port.kMXP);
    swerveDrive = new SwerveDrive(navX);
    intake = new Intake();
    feeder = new Feeder();
    shooter = new Shooter();

    //ifs = new IFSManual(intake, feeder, shooter, xbox);
    ifs = new IFSAuto(intake, feeder, shooter, xbox);

    irBreak = new DigitalInput(5);
    lights = new LED();

  }

  @Override
  public void robotInit() {
    swerveDrive.resetNavX();
  }

  @Override
  public void robotPeriodic() {
    // robotPosition = swerveDrive.updateOdometry();
    //check and make sure it works
    // SmartDashboard.putNumber("RobotXPos", robotPosition.getX());
    // SmartDashboard.putNumber("RobotYPos", robotPosition.getY());
    SmartDashboard.putNumber("Robot Angle", navX.getYaw());
    swerveDrive.displayPositions();
    Pose2d pose = swerveDrive.updateOdometry();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("R", pose.getRotation().getRotations());
    SmartDashboard.putBoolean("IR break", irBreak.get());
    

  }


  @Override
  public void teleopInit() {
    swerveDrive.resetOdometry();
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
      swerveDrive.resetNavX();
      navX.resetDisplacement();
    }
    x = -xAxis * Math.abs(xAxis);
    y = yAxis * Math.abs(yAxis);
    r = -rAxis * Math.abs(rAxis);

    swerveDrive.drive(x, y, r, true, true);

    if(xbox.getXButton()){
      swerveDrive.resetMaxVel();
      shooter.stopPivot();
    }

    ifs.update();
    
    if (irBreak.get()) {
      lights.teleopLights();
        
    }
    else{
      lights.noteDetectedLights();
    }
  }

  @Override
  public void autonomousInit() {
    // swerveDrive.resetNavX();
    // swerveDrive.setEncoders();
    // auto = BasicAuto.getStateMachine(feeder, shooter, swerveDrive);
    // auto.reset();

    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
    trajectory = new PathPlannerTraj("Diag", swerveDrive);
    timer.reset();

  }

  @Override
  public void autonomousPeriodic() {
    // auto.run();
    
    ifs.update();
    timer.start();
    SmartDashboard.putBoolean("Auto Done", trajectory.sample((long) (1000*timer.get())));

    lights.autoLights();

  }

  HolonomicDriveController follower;
  PIDController xController;
  PIDController yController;
  ProfiledPIDController rController;

  @Override
  public void testInit() {

    xController = new PIDController(0, 0, 0);
    xController.setP(0.025);
    xController.setI(0.01);
    xController.setD(0.0);

    yController = new PIDController(0, 0, 0);
    yController.setP(0.025);
    yController.setI(0.01);
    yController.setD(0.0);

    rController = new ProfiledPIDController(0, 0, 0, new Constraints(0.25, 0.25));
    rController.setP(0.5);
    rController.setI(0);
    rController.setD(0.0);

    follower = new HolonomicDriveController(
      xController,
      yController,
      rController
    );

  }


  @Override
  public void testPeriodic() {

    ChassisSpeeds speeds = follower.calculate(swerveDrive.getPose(), new State(), new Rotation2d());
    SmartDashboard.putNumber("VX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("VY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("W", speeds.omegaRadiansPerSecond);
    //swerveDrive.drive(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, true);

  }

  @Override
  public void disabledPeriodic() {
    lights.disabledLights();
  }

}