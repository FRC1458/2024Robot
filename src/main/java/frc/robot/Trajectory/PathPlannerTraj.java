package frc.robot.Trajectory;

import static frc.robot.RobotConstants.autoSpeed;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swervedrive.SwerveDrive;

public class PathPlannerTraj implements Trajectory {

    private final SwerveDrive swerveDrive;
    private final List<TimedEvent> timedEvents;
    private final PathPlannerTrajectory trajectory;

    @FunctionalInterface
    public static interface TimedEvent {
        public boolean attemptExecution(double time);
    }

    public static class WPICommandEvent implements TimedEvent {

        private final Command command;
        private final double executionTime;

        public WPICommandEvent(double executionTime, Command command) {
            this.command = command;
            this.executionTime = executionTime;
        }

        @Override
        public boolean attemptExecution(double time) {
            if (time > executionTime) {
                if (!command.isScheduled())
                    command.schedule();
                command.execute();
                return command.isFinished();
            }
            return false;
        }
        
    }

    public PathPlannerTraj(String name, SwerveDrive swerve) {

        swerveDrive = swerve;
        timedEvents = new ArrayList<>();
        trajectory = PathPlannerPath.fromPathFile(name).getTrajectory(new ChassisSpeeds(), swerve.navxAngle());

        trajectory.getEventCommands().forEach((Pair<Double, Command> x) -> timedEvents.add(new WPICommandEvent(x.getFirst(), x.getSecond())));
        
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

        double time = timestamp / 1000.0;
        if (time > trajectory.getTotalTimeSeconds()) {
            swerveDrive.drive(0, 0, 0, true, false);
            return true;
        }
        State state = trajectory.sample(time);

        double rot = state.heading.getRadians();
        double vx = state.velocityMps * Math.sin(rot);
        double vy = state.velocityMps * Math.cos(rot);

        SmartDashboard.putNumber("Velocity", state.velocityMps);
        SmartDashboard.putNumber("Heading", state.heading.getDegrees());
        
        SmartDashboard.putNumber("Desired X", state.positionMeters.getY());
        SmartDashboard.putNumber("Desired Y", state.positionMeters.getX());
        SmartDashboard.putNumber("Desired R", state.targetHolonomicRotation.getRotations());

        SmartDashboard.putNumber("Desired VX", vx);
        SmartDashboard.putNumber("Desired VY", vy);

    

        double ex = state.positionMeters.getY() - swerveDrive.getPose().getX();
        double ey = state.positionMeters.getX() - swerveDrive.getPose().getY();
        double er = state.targetHolonomicRotation.getRotations() - swerveDrive.getPose().getRotation().getRotations();

        SmartDashboard.putNumber("Error X", ex);
        SmartDashboard.putNumber("Error Y", ey);
        SmartDashboard.putNumber("Error R", er);

        swerveDrive.drive(
            vx / autoSpeed + 0.25 * ex,
            -vy / autoSpeed - 0.25 * ey,
            state.headingAngularVelocityRps / autoSpeed + 0 * er,
            false,
            false
        );

        for (int i = timedEvents.size() - 1; i >= 0; i--)
            if (timedEvents.get(i).attemptExecution(time))
                timedEvents.remove(i);

        return false;
    }
    
}