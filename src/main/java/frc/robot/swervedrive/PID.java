package frc.robot.swervedrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PID {
    private double kP;
    private double kI;
    private double kD;
    private double target = 0;
    private double iScaling = 20;
    
    private long previousTime = -1;
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
        long time = System.currentTimeMillis();
        accumError = accumError * (1 - 1/iScaling) + distance;
        
        if (previousTime == -1){
            previousTime = time - 20;
            previousDistance = distance;
        }

        double output = distance * kP + accumError * kI - (distance - previousDistance) / (time - previousTime) * kD;
        
        if(maxAccel > 0) {
            output = Math.min(Math.abs(output), Math.abs(previousOutput) + maxAccel) * Math.signum(output);
        }
        
        previousTime = time;    
        previousDistance  = distance;
        previousOutput = output;
        

        return output;
    }


    public double update(double current, double target) {
        setTarget(target);
        return update(current);
    }

    public void initDebug(String name) {
        SmartDashboard.putNumber(name + " P", 0);
        SmartDashboard.putNumber(name + " I", 0);
        SmartDashboard.putNumber(name + " D", 0);
        SmartDashboard.putNumber(name + " iScaling", 0);
    }

    public void updatePID(String name) {
        kP = SmartDashboard.getNumber(name + " P", 0);
        kI = SmartDashboard.getNumber(name + " I", 0);
        kD = SmartDashboard.getNumber(name + " D", 0);
        iScaling = SmartDashboard.getNumber(name + " iScaling", 0);
    }

}
