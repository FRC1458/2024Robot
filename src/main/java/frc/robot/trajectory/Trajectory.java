package frc.robot.trajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Trajectory {

    ChassisSpeeds getSpeeds();
    double getDuration();

}