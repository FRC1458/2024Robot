package frc.robot.trajectory;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SPathPlanner extends StateBasedTrajectory {

    // Requires correct Translation

    private final PathPlannerPath path;

    public SPathPlanner(String pathName, Supplier<State> stateSupplier) {
        super(stateSupplier);
        path = PathPlannerPath.fromPathFile(pathName);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        State state = stateSupplier.get();
        return stateToSpeeds(path.getTrajectory(state.getSpeeds(), state.getPose().getRotation()).getState(0));
    }

    @Override
    public double getDuration() {
        State state = stateSupplier.get();
        return path.getTrajectory(state.getSpeeds(), state.getPose().getRotation()).getTotalTimeSeconds();
    }

    private ChassisSpeeds stateToSpeeds(com.pathplanner.lib.path.PathPlannerTrajectory.State state) {
        double speed = state.velocityMps;
        Rotation2d angle = state.heading;
        Translation2d x = new Translation2d(speed, angle);
        return new ChassisSpeeds(x.getX(), x.getY(), state.headingAngularVelocityRps);
    }
    
}