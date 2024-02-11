package frc.robot.swervedrive;

public class PID {
    private double kP;
    private double kI;
    private double kD;
    private double target;
    private double iScaling = 20;
    
    private long previousTime = -1;
    private double previousDistance = -1;
    private double accumError = 0;
    

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
    
    public double update(double current){
        double distance = current - target;
        long time = System.currentTimeMillis();
        accumError = accumError * (1 - 1/iScaling) + distance;
        
        if (previousTime == -1){
            previousTime = time;
            previousDistance  = distance;    
        }

        double output = distance * kP + accumError * kI - (distance - previousDistance) / (time - previousTime) * kD;
        
        previousTime = time;    
        previousDistance  = distance;
        

        return output;
    }


    public double update(double current, double target) {
        setTarget(target);
        return update(current);
    }
}
