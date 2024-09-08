package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
 private CANSparkMax motor1; //right motor
 private CANSparkMax motor2; //left motor

 private RelativeEncoder encoder1;
 private RelativeEncoder encoder2;
 private SparkPIDController pidController1;
 private SparkPIDController pidController2;
 public static double SPEAKER_SPEED_RPM = 4000;
 public static  double AMP_SPEED_RPM = 2000;
 public static double TOLERANCE = 150;
 private static double KF = 0.000185;

 public Shooter(){
     motor1 = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
     motor2 = new CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless);

     motor1.restoreFactoryDefaults();
     motor2.restoreFactoryDefaults();

     pidController1 = motor1.getPIDController();
     pidController2 = motor2.getPIDController();

     pidController1.setP(RobotMap.KP12);
     pidController1.setI(0);
     pidController1.setD(0);
     pidController2.setP(RobotMap.KP14);
     pidController2.setI(0);
     pidController2.setD(0);
     pidController1.setFF(RobotMap.KF12);
     pidController2.setFF(RobotMap.KF14);

    pidController1.setOutputRange(-1,1);
    pidController2.setOutputRange(-1,1);

     encoder1 = motor1.getEncoder();
     encoder2 = motor2.getEncoder();

     motor1.setIdleMode(CANSparkBase.IdleMode.kCoast);
     motor2.setIdleMode(CANSparkBase.IdleMode.kCoast);

     SmartDashboard.putNumber("KF Shooter", KF);
 }

 public void moveFF(double speed){
  motor1.set(speed * KF);
  motor2.set(speed * KF);
 }

 public void movePid(double speed){
     pidController1.setReference(speed, CANSparkBase.ControlType.kVelocity);
     pidController2.setReference(speed, CANSparkBase.ControlType.kVelocity);
 }

 public void stop(){
  motor1.stopMotor();
  motor2.stopMotor();
 }

 public double getVelocity1(){
  return encoder1.getVelocity();
 }

 public double getVelocity2(){
  return encoder2.getVelocity();
 }

 public boolean isAtRPM1(double RPM){
    return MathUtil.isNear(RPM,encoder1.getVelocity(),TOLERANCE);
 }
 public boolean isAtRPM2(double RPM){
  return MathUtil.isNear(RPM,encoder2.getVelocity(),TOLERANCE);
 }



 public void resetPid(){
     pidController1.setIAccum(0);
     pidController2.setIAccum(0);
 }

 public void print(){
     SmartDashboard.putNumber("velocity 1", getVelocity1());
     SmartDashboard.putNumber("velocity 2", getVelocity2());
 }
}
