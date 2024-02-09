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


public class Autonomous {
    private Pose2d robotPos;
    private SwerveDrivePoseEstimator odometry;
    private Shooter shooter;
    private SwerveDrive swerve;

    public Autonomous(Pose2d robotPos, AHRS navX) {
        this.robotPos = robotPos;
        shooter = new Shooter();

        swerve = new Swerve(navX);
    }

    public thing() {
        shooter.shoot();
        swerve.drive(0.25, 0, 0, true); 
        
        Timer timer = new Timer();
        timer.start();

        while (true) {
            if (timer.hasElapsed(2)) {
                timer.stop();
                swerve.drive(0, 0, 0, true);
            }
        }
    }
}