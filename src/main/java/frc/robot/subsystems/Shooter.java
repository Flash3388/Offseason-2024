package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private CANSparkMax motorRight;
    private CANSparkMax motorLeft;

    private RelativeEncoder encoderRight;
    private RelativeEncoder encoderLeft;
    private SparkPIDController pidControllerRight;
    private SparkPIDController pidControllerLeft;
    public static double TOLERANCE = 150;
    private static double KF = 0.000185;

    public Shooter() {
        motorRight = new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
        motorLeft = new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT, CANSparkLowLevel.MotorType.kBrushless);

        motorRight.restoreFactoryDefaults();
        motorLeft.restoreFactoryDefaults();

        motorRight.setInverted(true);
        motorLeft.setInverted(true);

        pidControllerRight = motorRight.getPIDController();
        pidControllerLeft = motorLeft.getPIDController();

        pidControllerRight.setP(RobotMap.SHOOTER_RIGHT_KP1);
        pidControllerRight.setI(0);
        pidControllerRight.setD(0);
        pidControllerLeft.setP(RobotMap.SHOOTER_LEFT_KP1);
        pidControllerLeft.setI(0);
        pidControllerLeft.setD(0);
        pidControllerRight.setFF(RobotMap.SHOOTER_RIGHT_KP2);
        pidControllerLeft.setFF(RobotMap.SHOOTER_LEFT_KP2);

        pidControllerRight.setOutputRange(-1, 1);
        pidControllerLeft.setOutputRange(-1, 1);

        encoderRight = motorRight.getEncoder();
        encoderLeft = motorLeft.getEncoder();

        motorRight.setIdleMode(CANSparkBase.IdleMode.kCoast);
        motorLeft.setIdleMode(CANSparkBase.IdleMode.kCoast);

        SmartDashboard.putNumber("KF Shooter", KF);
    }

    public void moveFF(double speed) {
        motorRight.set(speed * KF);
        motorLeft.set(speed * KF);
    }

    public void movePid(double speed) {
        pidControllerRight.setReference(speed, CANSparkBase.ControlType.kVelocity);
        pidControllerLeft.setReference(speed, CANSparkBase.ControlType.kVelocity);
    }

    public void stop() {
        motorRight.stopMotor();
        motorLeft.stopMotor();
    }

    public double getVelocityRight() {
        return encoderRight.getVelocity();
    }

    public double getVelocityLeft() {
        return encoderLeft.getVelocity();
    }


    public void resetPid() {
        pidControllerRight.setIAccum(0);
        pidControllerLeft.setIAccum(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("velocity right", getVelocityRight());
        SmartDashboard.putNumber("velocity left", getVelocityLeft());
    }

    public boolean isAtRangePIDRight(double speed) {
        return (getVelocityRight() <= speed + TOLERANCE && getVelocityRight() >= speed - TOLERANCE);
    }

    public boolean isAtRangePIDLeft(double speed) {
        return (getVelocityLeft() <= speed + TOLERANCE && getVelocityLeft() >= speed - TOLERANCE);
    }

    public boolean isAtRangePID(double speed){
        return isAtRangePIDLeft(speed) && isAtRangePIDRight(speed);
    }
}
