package frc.robot.subsystems;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class SwerveModule {


    private static final double DRIVE_GEAR_RATIO = 6.75;
    private static final double STEER_GEAR_RATIO = 12.8;
    private static final double WHEEL_RADIUS = 0.0508;
    private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

    private static final double STEER_P = 0.59; // 0.05 //0.052 0053 0.054
    private static final double STEER_I = 0.00003; //0
    private static final double STEER_D = 0.01;
    private static final double STEER_F = 0.;
    private static final double STEER_IZONE = 5;

    private static final double DRIVE_P = 0.0003;//0.0003;
    private static final double DRIVE_I = 0;//5e-7;
    private static final double DRIVE_D = 0;//5e-7;
    private static final double DRIVE_F = 0.00016;//0.0001;

    private final CANSparkMax drive;
    private final CANSparkMax steer;
    private final CANcoder canCoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private final SparkPIDController drivePID;
    private final SparkPIDController steerPID;

    private final String printName;
    private final StatusSignal<Double> statusSignal;

    public SwerveModule(CANSparkMax drive,
                        CANSparkMax steer,
                        CANcoder canCoder,
                        double zeroAngle,
                        String identifier) {
        this.drive = drive;
        this.steer = steer;
        this.canCoder = canCoder;

        this.printName = String.format("SwerveModule[%s]", identifier);

        drive.setSmartCurrentLimit(20, 20);

        this.driveEncoder = this.drive.getEncoder();
        this.steerEncoder = this.steer.getEncoder();
        this.drivePID = this.drive.getPIDController();
        this.steerPID = this.steer.getPIDController();

        driveEncoder.setPosition(0);
        steerEncoder.setPosition(0);

        drivePID.setP(DRIVE_P);
        drivePID.setI(DRIVE_I);
        drivePID.setD(DRIVE_D);
        drivePID.setFF(DRIVE_F);
        drivePID.setOutputRange(-1, 1);

        steerPID.setP(STEER_P);
        steerPID.setI(STEER_I);
        steerPID.setD(STEER_D);
        steerPID.setFF(STEER_F);
        steerPID.setIZone(STEER_IZONE);
        steerPID.setOutputRange(-1, 1);
        steerPID.setPositionPIDWrappingMaxInput(STEER_GEAR_RATIO);
        steerPID.setPositionPIDWrappingMinInput(0);
        steerPID.setPositionPIDWrappingEnabled(true);

        statusSignal = canCoder.getAbsolutePosition();
        steerEncoder.setPosition((getAbsoluteHeadingDegrees() - zeroAngle) / 360 * STEER_GEAR_RATIO);
    }

    public double getHeadingDegrees() {
        double degrees = steerEncoder.getPosition() / STEER_GEAR_RATIO * 360;
        degrees %= 360;
        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    private double getAbsoluteHeadingDegrees() {
        statusSignal.refresh(true);
        return statusSignal.getValueAsDouble() * 360;
    }

    public double getVelocityRpm() {
        return driveEncoder.getVelocity() / DRIVE_GEAR_RATIO;
    }

    public double getVelocityMetersPerSecond() {
        return getVelocityRpm() / 60 * WHEEL_CIRCUMFERENCE;
    }

    public double getPositionMeters() {
        return driveEncoder.getPosition() / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    public Measure<Voltage> getOutputVoltage() {
        double voltage = drive.getBusVoltage() * drive.getAppliedOutput();
        return Volts.of(voltage);
    }

    public Measure<Velocity<Distance>> getLinearVelocity() {
        double velocity = getVelocityMetersPerSecond();
        return MetersPerSecond.of(velocity);
    }

    public SwerveModuleState getModuleStates() {
        return new SwerveModuleState(
                getVelocityMetersPerSecond(),
                getRotation()
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                getPositionMeters(),
                getRotation()
        );
    }

    public void resetEncoders() {
        steerEncoder.setPosition(0);
        driveEncoder.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation());
        setDriveVelocity(state.speedMetersPerSecond);
        setSteerPosition(state.angle.getDegrees());
    }

    public void setDriveVelocity(double velocityMps) {
        double driveVelocity = velocityMps * 60 / WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO;

        SmartDashboard.putNumber(printName + " Com-DriveRpm", driveVelocity);
        SmartDashboard.putNumber(printName + " Com-DriveMps", velocityMps);

        drivePID.setReference(driveVelocity, CANSparkBase.ControlType.kVelocity);
    }

    public void setSteerPosition(double positionDegrees) {
        double steerPosition = positionDegrees / 360.0 * STEER_GEAR_RATIO;
        double currentPositionStart = Math.floor(steerEncoder.getPosition() / STEER_GEAR_RATIO);
        double finalPosition = currentPositionStart * STEER_GEAR_RATIO + steerPosition;

        SmartDashboard.putNumber(printName + " Com-SteerRot", steerPosition);
        SmartDashboard.putNumber(printName + " Com-SteerCo", currentPositionStart);
        SmartDashboard.putNumber(printName + " Com-SteerF", finalPosition);
        SmartDashboard.putNumber(printName + " Com-SteerOrgDeg", positionDegrees);

        steerPID.setReference(finalPosition, CANSparkBase.ControlType.kPosition);
    }

    public void move(double drivespeed, double rotation) {
        drive.set(drivespeed);
        steer.set(rotation);
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public void periodic() {
        SmartDashboard.putNumber(printName + " Heading", getHeadingDegrees());
        SmartDashboard.putNumber(printName + " AbsHeading", getAbsoluteHeadingDegrees());
        SmartDashboard.putNumber(printName + " Pos", getPositionMeters());
        SmartDashboard.putNumber(printName + " Vel", getVelocityRpm());
        SmartDashboard.putNumber(printName + " VelOnM", driveEncoder.getVelocity());
        SmartDashboard.putNumber(printName + " VelMps", getVelocityMetersPerSecond());
        SmartDashboard.putNumber(printName + " DriveAmps", drive.getOutputCurrent());
        SmartDashboard.putNumber(printName + " SteerAmps", steer.getOutputCurrent());
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }

        return newAngle;
    }

}
