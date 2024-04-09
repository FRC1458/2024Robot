package frc.robot;

import com.fasterxml.jackson.core.sym.Name1;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Autos.CenterAuto;
import frc.robot.Autos.LongSideAuto;
import frc.robot.Autos.NonGoofyCenterAuto;
import frc.robot.Autos.ShortSideAuto.AutoStates;
import frc.robot.Trajectory.PathPlannerTraj;
import frc.robot.Trajectory.Trajectory;
import frc.robot.Trajectory.WPITraj;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.RobotConstants.*;

import java.util.ArrayList;
import java.util.List;


public class Robot extends TimedRobot {

  private final XboxController xbox1;
  private final XboxController xbox2;


  private final IFS ifs;

  public static DigitalInput irBreak;
  long time;
  Timer timer = new Timer();

  SwerveDrive swerveDrive;
  Pose2d robotPosition;
  Ultrasonic rangeFinder;

  Intake intake;
  Feeder feeder;
  Shooter shooter;


  private final AHRS navX;
  StateMachine<frc.robot.Autos.NonGoofyCenterAuto.AutoStates> auto;


  private Trajectory trajectory;


  private LED lights;


  public Robot() {
    super(0.02);
    xbox1 = new XboxController(0);
    xbox2 = new XboxController(1);
    time = System.currentTimeMillis();

    navX = new AHRS(SPI.Port.kMXP);
    swerveDrive = new SwerveDrive(navX);
    intake = new Intake();
    feeder = new Feeder();
    shooter = new Shooter();

    //ifs = new IFSManual(intake, feeder, shooter, xbox);
    ifs = new IFSAuto(intake, feeder, shooter, xbox1, xbox2);

    irBreak = new DigitalInput(5);
    lights = new LED();

  }

  @Override
  public void robotInit() {
    swerveDrive.resetNavX();
    SmartDashboard.putNumber("Rotation kP", 0.02);
  }

  @Override
  public void robotPeriodic() {
    // robotPosition = swerveDrive.updateOdometry();
    //check and make sure it works
    // SmartDashboard.putNumber("RobotXPos", robotPosition.getX());
    // SmartDashboard.putNumber("RobotYPos", robotPosition.getY());
    
    SmartDashboard.putNumber("Robot Angle", navX.getYaw());
    swerveDrive.displayPositions();
    //Pose2d pose = swerveDrive.updateOdometry();
    //SmartDashboard.putNumber("X", pose.getX());
    //SmartDashboard.putNumber("Y", pose.getY());
    //SmartDashboard.putNumber("R", pose.getRotation().getRotations());
    SmartDashboard.putBoolean("IR break", irBreak.get());
    intake.displayDiagnostics();
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

    xAxis = xbox1.getLeftX();
    yAxis = xbox1.getLeftY();
    rAxis = xbox1.getRightX();

    //check acceleration for acceleration limiter? otherwise can delete next 4 lines
    SmartDashboard.putNumber("X Acceleration", navX.getWorldLinearAccelX());
    SmartDashboard.putNumber("Y Acceleration", navX.getWorldLinearAccelY());
    SmartDashboard.putNumber("Z Acceleration", navX.getWorldLinearAccelZ());
    SmartDashboard.putNumber("Total xy Acceleration", Math.sqrt(Math.pow(navX.getWorldLinearAccelX(), 2) + Math.pow(navX.getWorldLinearAccelY(), 2)));

    if (xbox1.getStartButton()) {
      swerveDrive.setEncoders();
      swerveDrive.resetNavX();
      navX.resetDisplacement();
    }


    x = -xAxis * Math.abs(xAxis);
    y = yAxis * Math.abs(yAxis);
    r = -rAxis * Math.abs(rAxis);

    swerveDrive.driveRaw(x, y, r, true, true);

    if(xbox1.getXButton()){
      swerveDrive.resetMaxVel();
      shooter.stopPivot();
    }

    ifs.update();
    
    if(intake.tempHigh()){
      lights.intakeTempLights();
    }
    else if(ifs.isSource()) {
      lights.sourceLights();
    }
    else if (intake.isStalled()) {
      lights.intakeStallLights();
    }
    else if(ifs.isRampedUp()) {
      lights.rampedUpLights();
    }
    else if (ifs.isIntakeOverriden()) {
      lights.intakeOverrideLights();
    }
    else if (!irBreak.get()) {
      lights.noteDetectedLights();
    }
    else if (ifs.isIntakeActive()) {
      lights.intakeActiveLights();
    }
    else{
      lights.teleopLights();
    }

  }

  public static boolean noteDetected() {
    return !irBreak.get();
  }

  // private List<Trajectory> trajectories = new ArrayList<Trajectory>();

  @Override
  public void autonomousInit() {
    lights.autoLights();
    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
    timer.reset();

    // NamedCommands.registerCommand(
    //   "Intake",
    //   Commands
    //     .runOnce(intake::slurp)
    //     .andThen(feeder::assist)
    //     .until(Robot::noteDetected)
    //     .andThen(intake::stop)
    //     .andThen(feeder::stop)
    // );
    // trajectory = null;
    // trajectories.add(new PathPlannerTraj("InN2", swerveDrive));
    // trajectories.add(new PathPlannerTraj("OutN2", swerveDrive));
    // trajectories.add(new PathPlannerTraj("InN3", swerveDrive));
    // trajectories.add(new PathPlannerTraj("OutN3", swerveDrive));
    // trajectories.add(new PathPlannerTraj("InN1", swerveDrive));
    // trajectories.add(new PathPlannerTraj("OutN1", swerveDrive));

    // ShortSideAuto, CenterAuto, LongSideAuto
    // Color has to be "blue" or "red", case doesn't matter
    auto = NonGoofyCenterAuto.getStateMachine(intake, feeder, shooter, swerveDrive, "blue");
    auto.reset();

  }

  @Override
  public void autonomousPeriodic() {    

    intake.resetStallState();
    timer.start();

    // if (trajectory == null || trajectory.sample((long) (1000*timer.get()))) {
    //   if (trajectories.size() == 0) SmartDashboard.putBoolean("in Auto", false);
    //   else{ 
    //     trajectory = trajectories.remove(0);
    //     timer.reset();
    //   }
    // }



    auto.run();

  }

  HolonomicDriveController follower;
  PIDController xController;
  PIDController yController;
  ProfiledPIDController rController;

  @Override
  public void testInit() {
    /*
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
    */

  }


  @Override
  public void testPeriodic() {

    double err = Rotation2d.fromDegrees(0).minus(swerveDrive.navxAngle()).getDegrees();
    swerveDrive.drive(
        0,
        0,
        -SmartDashboard.getNumber("Rotation kP", 0) * err,
        true,
        false
    );

    //intake.displayConfig();
    //intake.runGoofyIntake();

 /*
    ChassisSpeeds speeds = follower.calculate(swerveDrive.getPose(), new State(), new Rotation2d());
    SmartDashboard.putNumber("VX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("VY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("W", speeds.omegaRadiansPerSecond);
    //swerveDrive.drive(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, true);
  */
  }

  @Override
  public void disabledPeriodic() {
    lights.disabledLights();
  }

}