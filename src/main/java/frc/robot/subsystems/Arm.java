package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final SparkLimitSwitch upperSwitch;
    private final SparkLimitSwitch lowerSwitch;

    private final DutyCycleEncoder encoder;
    private final SparkPIDController pidController;
    private final AbsoluteEncoder absoluteEncoder;

    public Arm() {
        leftMotor = new CANSparkMax(RobotMap.ARM_FOLLOW, CANSparkLowLevel.MotorType.kBrushless);
        rightMotor = new CANSparkMax(RobotMap.ARM_MASTER, CANSparkLowLevel.MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(80);

        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        upperSwitch = leftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        upperSwitch.enableLimitSwitch(true);
        lowerSwitch = leftMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        lowerSwitch.enableLimitSwitch(true);

        leftMotor.follow(rightMotor, true);

        pidController = leftMotor.getPIDController();

        encoder = new DutyCycleEncoder(RobotMap.ARM_ENCODER);

        absoluteEncoder = rightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setZeroOffset(RobotMap.OFFSET_TO_ZERO);
        absoluteEncoder.setPositionConversionFactor(360);
    }

    public void move(double speed){
        leftMotor.set(speed);
    }

    public void stop(){
        leftMotor.stopMotor();
    }

    public double getArmAngle(){
        return (encoder.getAbsolutePosition() - encoder.getPositionOffset()) * 360;
    }

    public boolean isAnyLimitSwitchActive(){
        return (upperSwitch.isLimitSwitchEnabled() || lowerSwitch.isLimitSwitchEnabled());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Offset", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Absolute Arm position", getArmAngle());
        SmartDashboard.putNumber("Arm Master Output", leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Arm Follower Output", rightMotor.getAppliedOutput());

        SmartDashboard.putBoolean("Any Switch Active", isAnyLimitSwitchActive());

        SmartDashboard.putNumber("Follower Set Velocity", rightMotor.get());
        SmartDashboard.putNumber("Master Set Velocity", leftMotor.get());

        SmartDashboard.putNumber("Absolute Encoder", absoluteEncoder.getPosition());
    }
}
