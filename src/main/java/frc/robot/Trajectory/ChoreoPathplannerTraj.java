/*
package frc.robot.Trajectory;

import static frc.robot.RobotConstants.autoAngVel;
import static frc.robot.RobotConstants.autoSpeed;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.swervedrive.SwerveDrive;

public class ChoreoPathplannerTraj implements Trajectory {

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

    public ChoreoPathplannerTraj(String name, SwerveDrive swerve) {

        swerveDrive = swerve;
        timedEvents = new ArrayList<>();
        trajectory = PathPlannerPath.fromChoreoTrajectory(name).getTrajectory(new ChassisSpeeds(), swerve.navxAngle().rotateBy(Rotation2d.fromDegrees(180)));

        //trajectory.getEventCommands().forEach((Pair<Double, Command> x) -> timedEvents.add(new WPICommandEvent(x.getFirst(), x.getSecond())));

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
            swerveDrive.drive(0, 0, 0, false, false);
            return true;
        }
        State state = trajectory.sample(time);

        double rot = state.heading.getRadians();
        double vx = state.velocityMps * Math.sin(rot);
        double vy = state.velocityMps * Math.cos(rot);
        double w = state.headingAngularVelocityRps;

        SmartDashboard.putNumber("Velocity", state.velocityMps);
        SmartDashboard.putNumber("Heading", state.heading.getDegrees());

        SmartDashboard.putNumber("Desired X", state.positionMeters.getY());
        SmartDashboard.putNumber("Desired Y", state.positionMeters.getX());
        SmartDashboard.putNumber("Desired R", state.targetHolonomicRotation.getDegrees());

        SmartDashboard.putNumber("Desired VX", vx);
        SmartDashboard.putNumber("Desired VY", vy);
        SmartDashboard.putNumber("Desired W", w);


        double ex = state.positionMeters.getY() - swerveDrive.getPose().getX();
        double ey = state.positionMeters.getX() - swerveDrive.getPose().getY();
        double er = state.targetHolonomicRotation.times(-1).rotateBy(Rotation2d.fromDegrees(180)).minus(swerveDrive.getPose().getRotation()).getRotations();

        SmartDashboard.putNumber("Error X", ex);
        SmartDashboard.putNumber("Error Y", ey);
        SmartDashboard.putNumber("Error R", er);

        double err = Rotation2d.fromDegrees(0).minus(swerveDrive.navxAngle()).getDegrees();
        swerveDrive.driveRaw(
                -vx / 3.074 - 0.0 * (ex * Math.signum(vx)),
                vy / 3.074 - 0.0 * (ey * Math.signum(vy)),
                -0.02 * err,
                true,
                false
        );

        for (int i = timedEvents.size() - 1; i >= 0; i--)
            if (timedEvents.get(i).attemptExecution(time))
                timedEvents.remove(i);

        return false;
    }

    @Override
    public boolean samplePosPID(long timestamp) {
        return false;
    }

}
 */