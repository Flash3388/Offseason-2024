package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private CANSparkMax motor;
    private SparkLimitSwitch forwardlimitswitch;
    private SparkLimitSwitch reverselimitswitch;
    private static final double SPEED = 0.5;

    public Climb() {
        motor=new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);

        forwardlimitswitch =  motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        reverselimitswitch = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        motor.restoreFactoryDefaults();
    }


    public void rotateMotor(boolean forwardOrReverse){
        double x = forwardOrReverse ? SPEED : -SPEED;
        motor.set(x);
    }
    public void stop(){
        motor.stopMotor();
    }
    public Boolean getf(){
        return forwardlimitswitch.isPressed();
    }

    public boolean getr(){
        return reverselimitswitch.isPressed();
    }
    public void print(){
        SmartDashboard.putBoolean("forwardlimitswitch",getf());
        SmartDashboard.putBoolean("reverselimitswitch", getr());
    }


}
