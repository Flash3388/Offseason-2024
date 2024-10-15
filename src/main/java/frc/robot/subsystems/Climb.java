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
    private static final double SPEED_FORWARD = 0.6;
    private static final double SPEED_DOWN = -1;

    public Climb() {
        motor = new CANSparkMax(RobotMap.CLIMB_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(80);

        forwardlimitswitch = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        reverselimitswitch = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    }


    public void rotateMotor(boolean forwardOrReverse) {
        double x = forwardOrReverse ? SPEED_FORWARD : SPEED_DOWN;
        motor.set(x);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean getForward() {
        return forwardlimitswitch.isPressed();
    }

    public boolean getReverse() {
        return reverselimitswitch.isPressed();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ClimbForwardLimitSwitch", getForward());
        SmartDashboard.putBoolean("ClimbReverseLimitSwitch", getReverse());
        SmartDashboard.putNumber("ClimbCurrent", motor.getOutputCurrent());
        SmartDashboard.putNumber("ClimbOutput", motor.getAppliedOutput());
    }
}
