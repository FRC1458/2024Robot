package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class TimeBasedTrajectory implements Trajectory {

    protected static double startTime = -1;

    public boolean isRunning() {
        return startTime != -1;
    }

    public void startTime() {
        startTime = getTime();
    }

    public ChassisSpeeds start() {
        startTime = getTime();
        return getSpeeds();
    }

    public void stop() {
        startTime = -1;
    }

    public double getCurrentTimestamp() {
        return getTime() - startTime;
    }

    protected double getTime() {
        return System.currentTimeMillis() / 1000.0;
    }
    
    public abstract Pose2d getInitialPose();
    
}