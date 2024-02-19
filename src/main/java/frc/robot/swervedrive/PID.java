package frc.robot.swervedrive;

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

    public void setMaxAccel(double speed){
        this.maxAccel = maxAccel;
    }


    
    public double update(double current){
        double distance = current - target;
        long time = System.currentTimeMillis();
        accumError = accumError * (1 - 1/iScaling) + distance;
        
        if (previousTime == -1){
            previousTime = time;
            previousDistance  = distance;    
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


}
