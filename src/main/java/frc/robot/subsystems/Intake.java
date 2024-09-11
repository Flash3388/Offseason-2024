package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
    private CANSparkMax motor;
    private DigitalInput left;
    private DigitalInput right;
    private static final double SPEED = 0.4;

    public Intake() {
        this.motor = new CANSparkMax(RobotMap.INTAKE_ID_MOTOR, CANSparkMax.MotorType.kBrushless);
        this.motor.restoreFactoryDefaults();
        this.left = new DigitalInput(RobotMap.INTAKE_ID_LEFT);
        this.right = new DigitalInput(RobotMap.INTAKE_ID_RIGHT);
        this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);


    }

    public void in() {
        this.motor.set(SPEED);
    }

    public void out() {
        this.motor.set(-SPEED);
    }

    public boolean getLeft() {
        return (!this.left.get());
    }

    public boolean getRight() {
        return (!this.right.get());
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public boolean hasBall() {
        return (getLeft() || getRight());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("digitalInputLeft", getLeft());
        SmartDashboard.putBoolean("digitalInputRight", getRight());
    }

    public void Move(double speed) {
        motor.set(speed);
    }
}
