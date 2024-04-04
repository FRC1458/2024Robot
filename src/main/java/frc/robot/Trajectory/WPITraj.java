package frc.robot.Trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

import static frc.robot.RobotConstants.autoSpeed;

import java.util.ArrayList;
import java.util.List;

public class WPITraj implements Trajectory {

    private final edu.wpi.first.math.trajectory.Trajectory trajectory;
    private final SwerveDrive swerveDrive;

    public WPITraj(SwerveDrive swerve) {
        // Positive = Right and Front
        List<ControlVector> controlVectors = new ArrayList<>();
        controlVectors.add(new ControlVector(new double[] {0,0,0}, new double[] {0,0.5,0}));
        controlVectors.add(new ControlVector(new double[] {0,0,0}, new double[] {2,0.5,0}));

        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.125);
        trajectory = TrajectoryGenerator.generateTrajectory(new ControlVectorList(controlVectors), config);
        swerveDrive = swerve;
        swerve.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public boolean sample(long timestamp) {
        if (timestamp / 1000.0 > trajectory.getTotalTimeSeconds()) return true;
        State state = trajectory.sample(timestamp / 1000.0);
        double rot = state.poseMeters.getRotation().getRadians();
        double vx = state.velocityMetersPerSecond * Math.cos(rot);
        double vy = state.velocityMetersPerSecond * Math.sin(rot);
        SmartDashboard.putNumber("Desired X", state.poseMeters.getY());
        SmartDashboard.putNumber("Desired Y", state.poseMeters.getX());
        SmartDashboard.putNumber("Desired R", state.poseMeters.getRotation().getRotations());
        swerveDrive.drive(-vx / autoSpeed, -vy / autoSpeed, 0, true, false);
        return false;
    }

    @Override
    public boolean samplePosPID(long timestamp) {
        return false;
    }
}