package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class Shooter extends Command {
 private CANSparkMax motor1;
 private CANSparkMax motor2;

 private RelativeEncoder encoder1;
 private RelativeEncoder encoder2;
 private SparkPIDController pidController1;
 private SparkPIDController pidController2;
 private double KP =0;
 private double KD=0;
 private double KI=0;
 public static double SPEAKER_SPEED_RPM = 4000;
 public static  double AMP_SPEED_RPM = 2000;
 public static double TOLERANCE = 250

 public Shooter(){
     motor1 = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
     motor2 = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);

     motor1.restoreFactoryDefaults();
     motor2.restoreFactoryDefaults();

     pidController1 = motor1.getPIDController();
     pidController2 = motor2.getPIDController();

     pidController1.setP(KP);
     pidController1.setI(KI);
     pidController1.setD(KD);
     pidController2.setP(KP);
     pidController2.setI(KI);
     pidController2.setD(KD);

     pidController1.setOutputRange(-1,1);
     pidController2.setOutputRange(-1,1);

     encoder1 = motor1.getEncoder();
     encoder2 = motor2.getEncoder();

     motor1.setIdleMode(CANSparkBase.IdleMode.kCoast);
     motor2.setIdleMode(CANSparkBase.IdleMode.kCoast);


 }

 public void moveFF(double speed){
  motor1.set(speed);
  motor2.set(speed);
 }

 public void movePid(double speed){
     pidController1.setReference(speed, CANSparkBase.ControlType.kVelocity);
     pidController2.setReference(speed, CANSparkBase.ControlType.kVelocity);
 }

 public void stop(){
  motor1.stopMotor();
  motor2.stopMotor();
 }

 public double getV1(){
  return encoder1.getVelocity();
 }

 public double getV2(){
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
}
