package frc.robot.Trajectory;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

import static frc.robot.RobotConstants.autoSpeed;
import static frc.robot.RobotConstants.autoAngVel;

public class ChoreoTraj implements Trajectory {

    private final ChoreoTrajectory trajectory;
    private final SwerveDrive swerveDrive;

    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController thetaPID;

    private double errorX;
    private double errorY;
    private double errorTheta;

    private Pose2d initialPose;
    private Pose2d currentPose;
    private SwerveDriveOdometry swerveOdometry;

    public ChoreoTraj(String name, SwerveDrive swerve) {
        trajectory = Choreo.getTrajectory(name);
        swerveDrive = swerve;
        xPID = new PIDController(1, 0, 0);
        yPID = new PIDController(1, 0, 0);
        thetaPID = new PIDController(0.1, 0, 0);
        initialPose = trajectory.getInitialPose();
        swerveOdometry = new SwerveDriveOdometry(swerveDrive.getKinematics(), swerveDrive.navxAngle(), swerveDrive.getPositions(), initialPose);
        //swerveDrive.resetOdometry(initialPose);
    }

    @Override
    public boolean sample(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTime()) return true;
        ChoreoTrajectoryState state = trajectory.sample(timestamp/1000.0);
        swerveDrive.drive(state.velocityX / autoSpeed, state.velocityY / autoSpeed, state.angularVelocity / autoAngVel, true, false);
        return false;
    }

    @Override
    public boolean samplePosPID(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTime()) {
            swerveDrive.drive(0, 0, 0, true, false);
            return true;
        }
        ChoreoTrajectoryState state = trajectory.sample(timestamp/1000.0);
        //swerveOdometry.update(swerveDrive.navxAngle(), swerveDrive.getPositions());
        //errorX = (-swerveOdometry.getPoseMeters().getX() - state.getPose().getX()) / 2;
        //errorY = (-swerveOdometry.getPoseMeters().getY() - state.getPose().getY()) / 2;
        //errorTheta = swerveOdometry.getPoseMeters().getRotation().getDegrees() - state.getPose().getRotation().getDegrees();
        errorX = (-swerveDrive.getPose().getY() - state.getPose().getX()) / 2;
        errorY = (-swerveDrive.getPose().getX() - state.getPose().getY()) / 2;
        errorTheta = swerveDrive.getPose().getRotation().getDegrees() - state.getPose().getRotation().getDegrees();
        SmartDashboard.putNumber("Error X", errorX);
        SmartDashboard.putNumber("Error Y", errorY);
        SmartDashboard.putNumber("Error R", errorTheta);

        //SmartDashboard.putNumber("Pose X", swerveOdometry.getPoseMeters().getX());
        //SmartDashboard.putNumber("Pose Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Pose X", swerveDrive.getPose().getX());
        SmartDashboard.putNumber("Pose Y", swerveDrive.getPose().getY());

        if (errorTheta > 180) errorTheta -= 180;
        if (errorTheta < -180) errorTheta += 180;
        swerveDrive.drive(yPID.calculate(errorY) * Math.signum(errorY), xPID.calculate(errorX) * Math.signum(errorX), thetaPID.calculate(errorTheta), true, false);
        return false;
    }
}
