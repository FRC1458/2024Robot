package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.StateMachine;

import static frc.robot.Autonomous.AutoState.*;


public class Autonomous {

    public enum AutoState{SPINSHOOTER, SHOOTSTORED, GOTONOTE1, SHOOTNOTE1, GOTONOTE2, SHOOTNOTE2}

    private SwerveDrivePoseEstimator odometry;
    private SwerveDrive swerve;
    private final Intake intake;
    private final Feeder feeder;
    private final Shooter shooter;


    private final StateMachine<AutoState> states = new StateMachine(SPINSHOOTER);

    private final Timer timer;
    public Autonomous(SwerveDrive swerve) {
        intake = new Intake();
        feeder = new Feeder();
        shooter = new Shooter();
        timer = new Timer();
    }

    public void autoInit() {
        states.addTimerState(SPINSHOOTER, 750, SHOOTSTORED, shooter::shoot);
        states.addTimerState(SHOOTSTORED, 250, GOTONOTE1, () -> {
            shooter.shoot();
            feeder.feed();
        });
    }


    public void thing() {
        if (!timer.hasElapsed(1)) {
            shooter.shoot();
        }
        if(timer.hasElapsed(0.75) && !timer.hasElapsed(1)) {
            feeder.feed();
        }
        if (timer.hasElapsed(1) && !timer.hasElapsed(4)) {
            swerve.drive(-0.02, 0, 0, true);
        }
        if (timer.hasElapsed(3) && !timer.hasElapsed(4)) {
            intake.slurp();
        }
        if (timer.hasElapsed(4.25) && !timer.hasElapsed(7.25)) {
            swerve.drive(0.02, 0, 0, true);
        }
        if (timer.hasElapsed(6.5) && !timer.hasElapsed(7.5)) {
            shooter.shoot();
        }
        if (timer.hasElapsed(7.25) && !timer.hasElapsed(7.5)) {
            feeder.feed();
        }
    }

}