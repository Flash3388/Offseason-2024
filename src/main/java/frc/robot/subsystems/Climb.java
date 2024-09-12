package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climb extends SubsystemBase {
    private CANSparkMax motor;
    private SparkLimitSwitch forwardlimitswitch;
    private SparkLimitSwitch reverselimitswitch;
    private static final double SPEED = 0.3;

    public Climb() {
        motor=new CANSparkMax(RobotMap.CLIMB_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

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

    public boolean getForward(){
        return forwardlimitswitch.isPressed();
    }

    public boolean getReverse(){
        return reverselimitswitch.isPressed();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ClimbForwardLimitSwitch", getForward());
        SmartDashboard.putBoolean("ClimbReverseLimitSwitch", getReverse());
    }
}
