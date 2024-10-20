package frc.robot;

public class TargetInfo {

    private double distance;
    private double angle;

    public TargetInfo(double distance, double angle){
        this.angle = angle;
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }

    @Override
    public String toString() {
        return "TargetInfo{" +
                "distance=" + distance +
                ", angle=" + angle +
                '}';
    }
}
