package frc.robot.Trajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Trajectory {

    ChassisSpeeds sample(long timestamp);
    
}