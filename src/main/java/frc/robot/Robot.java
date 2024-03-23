package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Trajectory.Trajectory;
import frc.robot.Trajectory.WPITraj;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;

public class Robot extends TimedRobot {
  


  private final XboxController xbox;

  private final IFS ifs;

  public static DigitalInput irBreak;
  int count;
  Timer timer;

  SwerveDrive swerveDrive;
  Pose2d robotPosition;
  Ultrasonic rangeFinder;

  Intake intake;
  Feeder feeder;
  Shooter shooter;


  private final AHRS navX;
  StateMachine<BasicAuto.AutoStates> auto;

  Trajectory trajectory;

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;


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

    irBreak = new DigitalInput(8);
    count = 0; //for LED's

  }

  @Override
  public void robotInit() {
    swerveDrive.resetNavX();
    led = new AddressableLED(1);
    ledBuffer = new AddressableLEDBuffer(120);
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
    SmartDashboard.putNumber("Robot Angle", navX.getYaw());
    swerveDrive.displayPositions();
    Pose2d pose = swerveDrive.updateOdometry();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    SmartDashboard.putNumber("R", pose.getRotation().getRotations());
    SmartDashboard.putBoolean("IR break", irBreak.get());
    
    if(!irBreak.get()) {
      for(int i = 0; i < ledBuffer.getLength();i++) {
        ledBuffer.setRGB(i, 0 , 255, 0);
      }
      led.setData(ledBuffer);
      count++;
    }

  }


  @Override
  public void teleopInit() {
    swerveDrive.setEncoders();
    swerveDrive.resetOdometry();
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

    swerveDrive.drive(x, y, r, true);

    if(xbox.getXButton()){
      swerveDrive.resetMaxVel();
      shooter.stopPivot();
    }

    ifs.update();
    
    if (irBreak.get()) {
      for(int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setHSV(i, (count + i) % 180, 255, 255);
      }
      led.setData(ledBuffer);
      count++;
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
    trajectory = new WPITraj(swerveDrive);
    timer.reset();

  }

  @Override
  public void autonomousPeriodic() {
    // auto.run();
       
    timer.start();
    SmartDashboard.putBoolean("Auto Done", trajectory.sample((long)(1000*timer.get())));

    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, (i / 5 + System.currentTimeMillis() / 1000) % 2 == 0 ? 255 : 0, 0, 0);
    led.setData(ledBuffer);

  }

  public TalonFX talon = new TalonFX(32);
  public TalonFXSim sim = new TalonFXSim(talon);
  public TalonFXConfiguration configs = new TalonFXConfiguration();
  public VelocityVoltage request = new VelocityVoltage(0);

  static class TalonFXSim {

    private final TalonFXSimState simState;

    private final double kS = 0.135;
    private final double kV = 0.090;
    private final double kA = 0.030;

    private double position = 0;
    private double velocity = 0;
    private double prev_velocity = 0;
    private Double prev_time = null;

    public TalonFXSim(TalonFX talon) {
      simState = talon.getSimState();
      simState.setSupplyVoltage(RobotController.getBatteryVoltage());
      simState.setRawRotorPosition(0);
    }

    public void update() {

      double v = simState.getMotorVoltage();

      prev_velocity = velocity;
      double time = System.currentTimeMillis() / 1000.0;
      double dt = (prev_time == null) ? Robot.kDefaultPeriod : time - prev_time;
      if (Math.abs(v) >= kS) velocity = (v - kS * Math.signum(v) + kA / dt * prev_velocity) / (kV + kA / dt);
      
      position += velocity * dt;
      simState.setRawRotorPosition(position);
      simState.setRotorVelocity(velocity);
      simState.setRotorAcceleration((velocity - prev_velocity) / dt);
      prev_time = time;
    }

  }

  static class ModuleSim {

    private final TalonFXSim angleSim;
    private final TalonFXSim driveSim;
    private final CANcoderSimState absEncSim;
    public ModuleSim(TalonFX angleTalon, TalonFX driveTalon, CANcoder absEnc) {
      angleSim = new TalonFXSim(angleTalon);
      driveSim = new TalonFXSim(driveTalon);
      absEncSim = absEnc.getSimState();
    }

  }

  @Override
  public void testInit() {

    // configs.MotionMagic
    //   .withMotionMagicCruiseVelocity(20)
    //   .withMotionMagicAcceleration(10)
    //   .withMotionMagicJerk(10);

    talon.getConfigurator().apply(configs);
    request.withSlot(0);

    SmartDashboard.putNumber("Talon kS", 0.135);
    SmartDashboard.putNumber("Talon kV", 0.09);
    SmartDashboard.putNumber("Talon kA", 0);
    SmartDashboard.putNumber("Talon P", 0.075);
    SmartDashboard.putNumber("Talon I", 0.1);
    SmartDashboard.putNumber("Talon D", 0);
    SmartDashboard.putNumber("Talon Target Velocity", 0);

  }

  @Override
  public void testPeriodic() {

    if (
      configs.Slot0.kS != SmartDashboard.getNumber("Talon kS", 0) ||
      configs.Slot0.kV != SmartDashboard.getNumber("Talon kV", 0) ||
      configs.Slot0.kA != SmartDashboard.getNumber("Talon kA", 0) ||
      configs.Slot0.kP != SmartDashboard.getNumber("Talon P", 0) ||
      configs.Slot0.kI != SmartDashboard.getNumber("Talon I", 0) ||
      configs.Slot0.kD != SmartDashboard.getNumber("Talon D", 0)
    ) {
      configs.Slot0
        .withKS(SmartDashboard.getNumber("Talon kS", 0))
        .withKV(SmartDashboard.getNumber("Talon kV", 0))
        .withKA(SmartDashboard.getNumber("Talon kA", 0))
        .withKP(SmartDashboard.getNumber("Talon P", 0))
        .withKI(SmartDashboard.getNumber("Talon I", 0))
        .withKD(SmartDashboard.getNumber("Talon D", 0));
      talon.getConfigurator().apply(configs);
    }

    talon.setControl(request.withVelocity(SmartDashboard.getNumber("Talon Target Velocity", 0)));

    SmartDashboard.putNumber("Talon Position", talon.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Talon Velocity", talon.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Talon Acceleration", talon.getAcceleration().getValueAsDouble());

    if (isSimulation()) sim.update();

  }

  @Override
  public void disabledPeriodic() {
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, 255, 80, 0);
    led.setData(ledBuffer);
  }

}