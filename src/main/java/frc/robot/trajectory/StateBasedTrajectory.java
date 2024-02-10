package frc.robot.trajectory;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class StateBasedTrajectory implements Trajectory {

    public static class State {

        private final Pose2d pose;
        private final ChassisSpeeds speeds;

        public State(Pose2d pose, ChassisSpeeds speeds) {
            this.pose = pose;
            this.speeds = speeds;
        }

        public Pose2d getPose() {
            return pose;
        }

        public ChassisSpeeds getSpeeds() {
            return speeds;
        }

    }

    protected final Supplier<State> stateSupplier;

    public StateBasedTrajectory(Supplier<State> stateSupplier) {
        this.stateSupplier = stateSupplier;
    }
    
}