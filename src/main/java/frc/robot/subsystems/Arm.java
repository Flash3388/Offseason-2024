package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
    private static final double KP = 0;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KF = 0;
    private static final double IZONE = 0;
    private final CANSparkMax followerMotor;
    private final CANSparkMax masterMotor;

    private final SparkLimitSwitch upperSwitch;
    private final SparkLimitSwitch lowerSwitch;

    private final SparkPIDController pidController;
    private final AbsoluteEncoder absoluteEncoder;


    public Arm() {
        followerMotor = new CANSparkMax(RobotMap.ARM_FOLLOW, CANSparkLowLevel.MotorType.kBrushless);
        masterMotor = new CANSparkMax(RobotMap.ARM_MASTER, CANSparkLowLevel.MotorType.kBrushless);

        followerMotor.restoreFactoryDefaults();
        masterMotor.restoreFactoryDefaults();

        followerMotor.setSmartCurrentLimit(80);
        masterMotor.setSmartCurrentLimit(80);

        followerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        masterMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        upperSwitch = followerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        upperSwitch.enableLimitSwitch(true);
        lowerSwitch = followerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        lowerSwitch.enableLimitSwitch(true);

        followerMotor.follow(masterMotor, true);

        absoluteEncoder = masterMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(360);

        pidController = masterMotor.getPIDController();
        pidController.setP(KP);
        pidController.setI(KI);
        pidController.setIZone(IZONE);
        pidController.setD(KD);
        pidController.setFF(KF);
        pidController.setFeedbackDevice(absoluteEncoder);

    }

    public void move(double speed){
        masterMotor.set(speed);
    }

    public void stop(){
        masterMotor.stopMotor();
    }

    public double getArmAngle(){
        return absoluteEncoder.getPosition();
    }

    public boolean isAnyLimitSwitchActive(){
        return (upperSwitch.isLimitSwitchEnabled() || lowerSwitch.isLimitSwitchEnabled());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Absolute Arm position", getArmAngle());
        SmartDashboard.putNumber("Arm Master Output", followerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Arm Follower Output", masterMotor.getAppliedOutput());

        SmartDashboard.putBoolean("Any Switch Active", isAnyLimitSwitchActive());

        SmartDashboard.putNumber("Follower Set Velocity", masterMotor.get());
        SmartDashboard.putNumber("Master Set Velocity", followerMotor.get());

        SmartDashboard.putNumber("Absolute Encoder", absoluteEncoder.getPosition());
    }
}
