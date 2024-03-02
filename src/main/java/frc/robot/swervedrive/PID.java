package frc.robot.swervedrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID {
    private double kP;
    private double kI;
    private double kD;
    private double target = 0;
    private double iScaling = 20;

    private double previousDistance = -1;
    private double accumError = 0;
    private double previousOutput = 0;
    private double maxAccel;
    

    public PID() {

    }


    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setTarget(double target){
        this.target = target;
    }

    public void setiScaling(double iScaling) {
        this.iScaling = iScaling;
    }

    public void setMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel * 0.02;
    }


    
    public double update(double current){
        double distance = current - target;
        accumError = accumError * (1 - 1 / Math.max(1, iScaling)) + distance;
        
        if (previousDistance == -1) {
            previousDistance = distance;
        }

        double output = distance * kP + accumError * kI - (distance - previousDistance) / 20 * kD;
        
        if(maxAccel > 0) {
            SmartDashboard.putNumber("Output", output);
            SmartDashboard.putNumber("Prev Output", previousOutput);
            SmartDashboard.putNumber("Max Accel", maxAccel); // 500
            SmartDashboard.putNumber("Block 2", Math.abs(previousOutput) + maxAccel);
            output = Math.min(Math.abs(output), Math.abs(previousOutput) + maxAccel) * Math.signum(output);
            SmartDashboard.putNumber("Final Output", output);
        }

        previousDistance = distance;
        previousOutput = output;
        SmartDashboard.putNumber("Final Prev Output", previousOutput);

        return output;
    }


    public double update(double current, double target) {
        setTarget(target);
        return update(current);
    }

    public void initDebug(String name) {
        SmartDashboard.putNumber(name + " P", kP);
        SmartDashboard.putNumber(name + " I", kI);
        SmartDashboard.putNumber(name + " D", kD);
        SmartDashboard.putNumber(name + " iScaling", iScaling);
    }

    public void updatePID(String name) {
        kP = SmartDashboard.getNumber(name + " P", kP);
        kI = SmartDashboard.getNumber(name + " I", kI);
        kD = SmartDashboard.getNumber(name + " D", kD);
        iScaling = SmartDashboard.getNumber(name + " iScaling", iScaling);
    }

}
