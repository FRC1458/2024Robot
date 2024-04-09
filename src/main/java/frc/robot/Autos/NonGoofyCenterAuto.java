package frc.robot.Autos;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Feeder;
import frc.robot.Intake;
import frc.robot.Robot;
import frc.robot.Shooter;
import frc.robot.Trajectory.PathPlannerTraj;
import frc.robot.Trajectory.Trajectory;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.util.StateMachine;
import static frc.robot.Autos.NonGoofyCenterAuto.AutoStates.*;

public class NonGoofyCenterAuto {

    public enum AutoStates {RESET_ENCODERS, SHOOT0, IN1, OUT1, SHOOT1, IN2, OUT2, SHOOT2, IN3, OUT3, SHOOT3, TAXI, END}

    public static StateMachine<AutoStates> getStateMachine(Intake intake, Feeder feeder, Shooter shooter, SwerveDrive swerveDrive, String color) {

        List<Trajectory> trajectories = new ArrayList<>();
        trajectories.add(new PathPlannerTraj("InN2", swerveDrive));
        trajectories.add(new PathPlannerTraj("OutN2", swerveDrive));
        trajectories.add(new PathPlannerTraj("InN3", swerveDrive));
        trajectories.add(new PathPlannerTraj("OutN3", swerveDrive));
        trajectories.add(new PathPlannerTraj("InN1", swerveDrive));
        trajectories.add(new PathPlannerTraj("OutN1", swerveDrive));

        StateMachine<AutoStates> stateMachine = new StateMachine<>(RESET_ENCODERS);
        stateMachine.addBoolState(RESET_ENCODERS, SHOOT0, () -> {
            swerveDrive.resetNavX();
            swerveDrive.setEncoders();
            return true;
        });
        stateMachine.addBoolState(SHOOT0, IN1, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });

        stateMachine.addBoolState(IN1, OUT1, (ts) -> {
            if (ts > 900 && Robot.irBreak.get()) {
                feeder.feed();
                intake.slurp();
            } else {
                feeder.stop();
                intake.stop();
            }
            shooter.stop();
            return trajectories.get(0).sample(ts);
        });
        stateMachine.addBoolState(OUT1, SHOOT1, (ts) -> {
            feeder.stop();
            intake.stop();
            shooter.shootSpeaker();
            return trajectories.get(1).sample(ts);
        });
        stateMachine.addBoolState(SHOOT1, IN2, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });

        stateMachine.addBoolState(IN2, OUT2, (ts) -> {
            if (ts > 1250 && Robot.irBreak.get()) {
                feeder.feed();
                intake.slurp();
            } else {
                feeder.stop();
                intake.stop();
            }
            shooter.stop();
            return trajectories.get(2).sample(ts);
        });
        stateMachine.addBoolState(OUT2, SHOOT2, (ts) -> {
            feeder.stop();
            intake.stop();
            shooter.shootSpeaker();
            return trajectories.get(3).sample(ts);
        });
        stateMachine.addBoolState(SHOOT2, IN3, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });

        stateMachine.addBoolState(IN3, OUT3, (ts) -> {
            if (ts > 1350 && Robot.irBreak.get()) {
                feeder.feed();
                intake.slurp();
            } else {
                feeder.stop();
                intake.stop();
            }
            shooter.stop();
            return trajectories.get(4).sample(ts);
        });
        stateMachine.addBoolState(OUT3, SHOOT3, (ts) -> {
            feeder.stop();
            intake.stop();
            shooter.shootSpeaker();
            return trajectories.get(5).sample(ts);
        });
        stateMachine.addBoolState(SHOOT3, END, () -> {
            shooter.shootSpeaker();
            feeder.feed();
            intake.slurp();
            return Robot.irBreak.get();
        });

        stateMachine.addOffState(END, () -> {
            shooter.stop();
            feeder.stop();
            intake.stop();
            swerveDrive.stop();
        });
        return stateMachine;

    }

}