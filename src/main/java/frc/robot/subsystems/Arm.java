package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {

    private static final double KP = 0.02;
    private static final double KI = 0;
    private static final double KD = 0;
    private static final double KF = 0;
    private static final double IZONE = 0;
    private static final double TOLERANCE_DEGREES = 3;
    private static final double TOLERANCE_VELOCITY_RPM = 1;
    private static final int POSITION_PID_SLOT = 0;
    private static final double HIGH_CURRENT_TO_WARN = 40;
    private static final double HIGH_TEMPERATURE_TO_WARN = 70;

    private final CANSparkMax followerMotor;
    private final CANSparkMax masterMotor;

    private final SparkLimitSwitch upperSwitch;
    private final SparkLimitSwitch lowerSwitch;

    private final SparkPIDController pidController;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;


    public Arm() {
        followerMotor = new CANSparkMax(RobotMap.ARM_FOLLOW, CANSparkLowLevel.MotorType.kBrushless);
        masterMotor = new CANSparkMax(RobotMap.ARM_MASTER, CANSparkLowLevel.MotorType.kBrushless);

        followerMotor.restoreFactoryDefaults();
        masterMotor.restoreFactoryDefaults();

        followerMotor.setSmartCurrentLimit(60);
        masterMotor.setSmartCurrentLimit(60);

        followerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        masterMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        masterMotor.setInverted(true);

        upperSwitch = followerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        upperSwitch.enableLimitSwitch(true);
        lowerSwitch = followerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        lowerSwitch.enableLimitSwitch(true);

        followerMotor.follow(masterMotor, true);

        absoluteEncoder = masterMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(360);

        relativeEncoder = masterMotor.getEncoder();

        pidController = masterMotor.getPIDController();
        pidController.setP(KP, POSITION_PID_SLOT);
        pidController.setI(KI, POSITION_PID_SLOT);
        pidController.setIZone(IZONE, POSITION_PID_SLOT);
        pidController.setD(KD, POSITION_PID_SLOT);
        pidController.setFF(KF, POSITION_PID_SLOT);
        pidController.setFeedbackDevice(absoluteEncoder);

        masterMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        masterMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
        masterMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) RobotMap.ARM_CEILING_ANGLE);
        masterMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) RobotMap.ARM_FLOOR_ANGLE);
    }

    public void move(double speed) {
        masterMotor.set(speed);
    }

    public void setMoveToPosition(double positionDegrees) {
        // filter out noise by just looking at the integer component, this is fine, and we don't need
        // better precision than this for FF. No point using capable filters.
        double currentPosition = (int) (getAngleDegrees() - RobotMap.ARM_FLOOR_ANGLE);
        double arbFeedForward = Math.cos(Math.toRadians(currentPosition)) * 0.1;

        SmartDashboard.putNumber("ArmFFDeg", currentPosition);
        SmartDashboard.putNumber("ArmArbFF", arbFeedForward);

        pidController.setReference(positionDegrees, CANSparkBase.ControlType.kPosition,
                POSITION_PID_SLOT,
                arbFeedForward,
                SparkPIDController.ArbFFUnits.kPercentOut);
    }

    public void stop() {
        masterMotor.stopMotor();
    }

    public double getAngleDegrees() {
        return absoluteEncoder.getPosition();
    }

    public double getVelocityRpm() {
        return relativeEncoder.getVelocity();
    }

    public boolean isAnyLimitSwitchActive() {
        return (upperSwitch.isLimitSwitchEnabled() || lowerSwitch.isLimitSwitchEnabled());
    }

    public boolean didReachTarget(double position) {
        return MathUtil.isNear(position, getAngleDegrees(), TOLERANCE_DEGREES) && Math.abs(getVelocityRpm()) < TOLERANCE_VELOCITY_RPM;
    }

    public boolean isAtFloor() {
        // add +1 to angle to give it a margin of +1 degrees
        return getAngleDegrees() <= RobotMap.ARM_FLOOR_ANGLE + 1 && Math.abs(getVelocityRpm()) < TOLERANCE_VELOCITY_RPM;
    }

    @Override
    public void periodic() {
        double currentMaster = Math.abs(masterMotor.getOutputCurrent());
        double currentFollower = Math.abs(followerMotor.getOutputCurrent());

        double tempMaster = masterMotor.getMotorTemperature();
        double tempFollower = followerMotor.getMotorTemperature();

        SmartDashboard.putNumber("ArmMasterCurrent", currentMaster);
        SmartDashboard.putNumber("ArmFollowerCurrent", currentFollower);
        SmartDashboard.putNumber("ArmMasterTemp", tempMaster);
        SmartDashboard.putNumber("ArmFollowerTemp", tempFollower);
        SmartDashboard.putNumber("ArmPosition", getAngleDegrees());
        SmartDashboard.putNumber("ArmVelocityRpm", getVelocityRpm());
        SmartDashboard.putBoolean("ArmSwitchActive", isAnyLimitSwitchActive());

        boolean shouldWarnCurrent = currentFollower > HIGH_CURRENT_TO_WARN || currentMaster > HIGH_CURRENT_TO_WARN;
        SmartDashboard.putBoolean("ArmHighMotorCurrent", shouldWarnCurrent);

        boolean shouldWarnTemp = tempMaster > HIGH_TEMPERATURE_TO_WARN || tempFollower > HIGH_TEMPERATURE_TO_WARN;
        SmartDashboard.putBoolean("ArmHighMotorTemp", shouldWarnTemp);
    }
}
