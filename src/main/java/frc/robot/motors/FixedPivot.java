package frc.robot.motors;

public class FixedPivot extends Pivot {

    private final double position;

    public FixedPivot(double position) {
        this.position = position;
    }

    @Override
    public void setSpeed(double speed) {}

    @Override
    public void hold() {}

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public void stop() {}
    
}