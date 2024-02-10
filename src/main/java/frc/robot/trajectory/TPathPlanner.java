package frc.robot.trajectory;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.trajectory.StateBasedTrajectory.State;

public class TPathPlanner extends TimeBasedTrajectory {

    // Requires correct Translation

    private final PathPlannerTrajectory trajectory;

    public TPathPlanner(String pathName, State state) {
        trajectory = PathPlannerPath.fromPathFile(pathName).getTrajectory(state.getSpeeds(), state.getPose().getRotation());
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return stateToSpeeds(trajectory.sample(getCurrentTimestamp()));
    }

    @Override
    public double getDuration() {
        return trajectory.getTotalTimeSeconds();
    }

    @Override
    public Pose2d getInitialPose() {
        return trajectory.getInitialTargetHolonomicPose();
    }

    private ChassisSpeeds stateToSpeeds(com.pathplanner.lib.path.PathPlannerTrajectory.State state) {
        double speed = state.velocityMps;
        Rotation2d angle = state.heading;
        Translation2d x = new Translation2d(speed, angle);
        return new ChassisSpeeds(x.getX(), x.getY(), state.headingAngularVelocityRps);
    }

}