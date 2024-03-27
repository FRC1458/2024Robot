package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class TalonFXSim {

    private final TalonFXSimState simState;

    private final double kS = 0.135;
    private final double kV = 0.090;
    private final double kA = 0.030;

    private double position = 0;
    private double velocity = 0;
    private double prev_velocity = 0;
    private Double prev_time = null;

    public TalonFXSim(TalonFX talon) {
      simState = talon.getSimState();
      simState.setSupplyVoltage(RobotController.getBatteryVoltage());
      simState.setRawRotorPosition(0);
    }

    public void update() {

      double v = simState.getMotorVoltage();

      prev_velocity = velocity;
      double time = System.currentTimeMillis() / 1000.0;
      double dt = (prev_time == null) ? Robot.kDefaultPeriod : time - prev_time;
      if (Math.abs(v) >= kS) velocity = (v - kS * Math.signum(v) + kA / dt * prev_velocity) / (kV + kA / dt);
      
      position += velocity * dt;
      simState.setRawRotorPosition(position);
      simState.setRotorVelocity(velocity);
      simState.setRotorAcceleration((velocity - prev_velocity) / dt);
      prev_time = time;
    }

  }