package frc.robot.Trajectory;

import static frc.robot.RobotConstants.autoSpeed;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class PathPlannerTraj implements Trajectory {

    private final SwerveDrive swerveDrive;
    private final PathPlannerTrajectory trajectory;

    public PathPlannerTraj(String name, SwerveDrive swerve) {

        swerveDrive = swerve;
        trajectory = PathPlannerPath.fromPathFile(name).getTrajectory(new ChassisSpeeds(), swerve.navxAngle());
        swerveDrive.resetOdometry(
            new Pose2d(
                trajectory.getInitialTargetHolonomicPose().getY(),
                trajectory.getInitialTargetHolonomicPose().getX(),
                trajectory.getInitialTargetHolonomicPose().getRotation()
            )
        );

    }

    @Override
    public boolean sample(long timestamp) {

        if (timestamp / 1000.0 > trajectory.getTotalTimeSeconds()) {
            swerveDrive.drive(0, 0, 0, true, false);
            return true;
        } 
        State state = trajectory.sample(timestamp / 1000.0);

        double rot = state.heading.getRotations();
        double vx = state.velocityMps * Math.sin(rot);
        double vy = state.velocityMps * Math.cos(rot);
        

        SmartDashboard.putNumber("Desired X", state.positionMeters.getY());
        SmartDashboard.putNumber("Desired Y", state.positionMeters.getX());
        SmartDashboard.putNumber("Desired R", state.targetHolonomicRotation.getRotations());

        SmartDashboard.putNumber("D. X", vx);
        SmartDashboard.putNumber("D. Y", vy);

        

        double ex = state.positionMeters.getY() - swerveDrive.getPose().getX();
        double ey = state.positionMeters.getX() - swerveDrive.getPose().getY();
        double er = state.targetHolonomicRotation.getRotations() - swerveDrive.getPose().getRotation().getRotations();

        SmartDashboard.putNumber("Error X", ex);
        SmartDashboard.putNumber("Error Y", ey);
        SmartDashboard.putNumber("Error R", er);

        swerveDrive.drive(
            -vx / autoSpeed,// + 0.25 * ex,
            -vy / autoSpeed,// - 0.25 * ey,
            0 * er,
            false,
            false
        );

        return false;
    }
    
}