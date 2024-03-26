package frc.robot.Trajectory;

import static frc.robot.RobotConstants.autoSpeed;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class PathPlannerTraj implements Trajectory {

    private final SwerveDrive swerveDrive;
    private final PathPlannerTrajectory trajectory;

    public PathPlannerTraj(String name, SwerveDrive swerve) {
        swerveDrive = swerve;
        trajectory = PathPlannerPath.fromPathFile(name).getTrajectory(new ChassisSpeeds(), swerve.navxAngle());
        swerveDrive.resetOdometry(
            new Pose2d(
                trajectory.getInitialTargetHolonomicPose().getY(),
                trajectory.getInitialTargetHolonomicPose().getX(),
                trajectory.getInitialTargetHolonomicPose().getRotation()
            )
        );
        SmartDashboard.putNumber("Trajectory P", 0);
        SmartDashboard.putNumber("Trajectory I", 0);
        SmartDashboard.putNumber("Trajectory D", 0);
    }

    @Override
    public boolean sample(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTimeSeconds()) return true;
        State state = trajectory.sample(timestamp / 1000.0);

        double rot = state.heading.getRotations();
        double vx = state.velocityMps * Math.sin(rot);
        double vy = state.velocityMps * Math.cos(rot);
        
        SmartDashboard.putNumber("Desired X", state.positionMeters.getY());
        SmartDashboard.putNumber("Desired Y", state.positionMeters.getX());
        SmartDashboard.putNumber("Desired R", state.targetHolonomicRotation.getRotations());

        double ex = state.positionMeters.getY() - swerveDrive.getPose().getX();
        double ey = state.positionMeters.getX() - swerveDrive.getPose().getY();
        double er = state.targetHolonomicRotation.getRotations() - swerveDrive.getPose().getRotation().getRotations();

        SmartDashboard.putNumber("Error X", ex);
        SmartDashboard.putNumber("Error Y", ey);
        SmartDashboard.putNumber("Error R", er);

        double p = SmartDashboard.getNumber("Trajectory P", 0);
        double i = SmartDashboard.getNumber("Trajectory I", 0);
        double d = SmartDashboard.getNumber("Trajectory D", 0);

        swerveDrive.drive(-vx / autoSpeed + p * ex, -vy / autoSpeed + p * ey, -0.1 * er, true);
        return false;
    }
    
}