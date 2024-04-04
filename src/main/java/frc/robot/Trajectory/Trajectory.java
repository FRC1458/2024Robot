package frc.robot.Trajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Trajectory {

    boolean sample(long timestamp);

    boolean samplePosPID(long timestamp);
}