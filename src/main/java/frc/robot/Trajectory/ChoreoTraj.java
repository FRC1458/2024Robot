package frc.robot.Trajectory;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

import static frc.robot.RobotConstants.autoSpeed;
import static frc.robot.RobotConstants.autoAngVel;

public class ChoreoTraj implements Trajectory {

    private final ChoreoTrajectory trajectory;
    private final SwerveDrive swerveDrive;

    public ChoreoTraj(String name, SwerveDrive swerve) {
        trajectory = Choreo.getTrajectory(name);
        swerveDrive = swerve;
    }

    @Override
    public boolean sample(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTime()) return true;
        ChoreoTrajectoryState state = trajectory.sample(timestamp/1000.0);
        swerveDrive.drive(state.velocityY / autoSpeed, -state.velocityX / autoSpeed, state.angularVelocity / autoAngVel, true);
        return false;
    }
}