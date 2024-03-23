package frc.robot.Trajectory;

import static frc.robot.RobotConstants.autoSpeed;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.swervedrive.SwerveDrive;

public class PathPlannerTraj implements Trajectory {

    private final SwerveDrive swerveDrive;
    private final PathPlannerTrajectory trajectory;

    public PathPlannerTraj(String name, SwerveDrive swerve) {
        swerveDrive = swerve;
        trajectory = PathPlannerPath.fromPathFile(name).getTrajectory(new ChassisSpeeds(), swerve.navxAngle());
    }

    @Override
    public boolean sample(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTimeSeconds()) return true;
        State state = trajectory.sample(timestamp / 1000.0);
        double rot = state.heading.getRotations();
        double vx = state.velocityMps * Math.cos(rot);
        double vy = state.velocityMps * Math.sin(rot);
        swerveDrive.drive(-vx / autoSpeed, -vy / autoSpeed, 0, true);
        return false;
    }
    
}