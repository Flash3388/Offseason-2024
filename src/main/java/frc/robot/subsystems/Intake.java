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
    private static final double speedIn = 0.3;
    private static final double speedOut = -0.3;

    public Intake(){
        this.motor.restoreFactoryDefaults();
        this.motor = new CANSparkMax(RobotMap.ID_MOTOR, CANSparkMax.MotorType.kBrushless);
        this.left = new DigitalInput(RobotMap.ID_LEFT);
        this.right = new DigitalInput(RobotMap.ID_RIGHT);
        this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);


    }

    public void In(){
        this.motor.set(speedIn);
    }

    public void out(){
        this.motor.set(speedOut);
    }

    public boolean getLeft(){
        return this.left.get();
    }

    public boolean getRight(){
        return this.right.get();
    }

    public void stop(){
        this.motor.stopMotor();
    }

    public void print(){
        SmartDashboard.putBoolean("digitalInputLeft", left.get());
        SmartDashboard.putBoolean("digitalInputRight", right.get());
    }
}
