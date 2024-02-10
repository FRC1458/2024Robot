package frc.robot.trajectory;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TChoreo extends TimeBasedTrajectory {

    // Requires correct Pose

    private final ChoreoTrajectory trajectory;

    public TChoreo(String trajName) {
        trajectory = Choreo.getTrajectory(trajName);
    }

    public ChassisSpeeds getSpeeds() {
        if (startTime == -1) return null;
        return trajectory.sample(getCurrentTimestamp()).getChassisSpeeds();
    }

    public double getDuration() {
        return trajectory.getTotalTime();
    }

    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

}