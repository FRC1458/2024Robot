package frc.robot.Trajectory;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
        xPID = new PIDController(0.1, 0, 0);
        yPID = new PIDController(0.1, 0, 0);
        thetaPID = new PIDController(0.1, 0, 0);
        initialPose = trajectory.getInitialPose();
        swerveOdometry = new SwerveDriveOdometry(swerveDrive.getKinematics(), swerveDrive.navxAngle(), swerveDrive.getPositions(), initialPose);
    }

    @Override
    public boolean sample(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTime()) return true;
        ChoreoTrajectoryState state = trajectory.sample(timestamp/1000.0);
        swerveDrive.drive(state.velocityX / autoSpeed, state.velocityY / autoSpeed, state.angularVelocity / autoAngVel, true);
        return false;
    }

    public boolean sampleWithPositionPIDs(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTime()) return true;
        ChoreoTrajectoryState state = trajectory.sample(timestamp/1000.0);
        currentPose = swerveOdometry.update(swerveDrive.navxAngle(), swerveDrive.getPositions());
        errorX = state.x - currentPose.getX();
        errorY = state.y - currentPose.getY();
        errorTheta = state.heading - currentPose.getRotation().getRadians();
        if (errorTheta > Math.PI ) errorTheta -= 2 * Math.PI;
        else if (errorTheta < -Math.PI) errorTheta += 2 * Math.PI;
        swerveDrive.drive(xPID.calculate(errorX), yPID.calculate(errorY), thetaPID.calculate(errorTheta), true);
        return false;
    }
}