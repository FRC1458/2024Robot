package frc.robot.trajectory;

import java.io.File;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.trajectory.StateBasedTrajectory.State;

public class TrajectoryDatabase {

    private final HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();
    private final Supplier<State> nullStateSupplier = new Supplier<StateBasedTrajectory.State>() {
        @Override
        public State get() {
            return new State(new Pose2d(), new ChassisSpeeds());
        }
    };

    public TrajectoryDatabase() {

        // Load Choreo Trajectories (Only Time-Based)
        for (File file: new File("src\\main\\deploy\\choreo").listFiles()) {
            try {
                String filename = file.getName();
                if (!filename.endsWith(".traj")) continue;
                String trajName = filename.replace(".traj", "");
                if (trajName.contains(".")) continue;
                trajectories.put("T" + trajName, new TChoreo(trajName));
            } catch (Exception e) {System.out.println("!!! Issue with Choreo Trajectory!");}
        }

        // Load Path Planner Trajectories (State-Based or Time-Based)
        for (File file: new File("src\\main\\deploy\\choreo").listFiles()) {
            try {
                String filename = file.getName();
                if (!filename.endsWith(".path")) continue;
                String pathName = filename.replace(".path", "");
                trajectories.put(pathName, new SPathPlanner("S" + pathName, nullStateSupplier));
                trajectories.put(pathName, new TPathPlanner("T" + pathName, nullStateSupplier.get()));
            } catch (Exception e) {System.out.println("!!! Issue with Path Planner Trajectory!");}
        }

        // Load PathWeaver Trajectories
        // for (File file: new File("src\\main\\deploy\\...").listFiles()) {
        //     String filename = file.getName();
        //     if (!filename.endsWith(".XXX")) continue;
        //     try {
        //         TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(filename));
        //     } catch (Exception e) {System.out.println("!!! Issue with WPILib Trajectory!");}
        // }

    }

    public Trajectory getTrajectory(String trajName) {
        return trajectories.get(trajName);
    }

}