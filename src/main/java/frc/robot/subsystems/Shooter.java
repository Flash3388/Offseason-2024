package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
 public static double speakerSPEED = 4000;
 public static  double ampSPEED = 2000;

 public Shooter(){
     motor1 = new CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless);
     motor2 = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless);
     pidController1 = motor1.getPIDController();
     pidController2 = motor2.getPIDController();
     pidController1.setP(KP);
     pidController1.setI(KI);
     pidController1.setD(KD);
     pidController2.setP(KP);
     pidController2.setI(KI);
     pidController2.setD(KD);
     encoder1 = motor1.getEncoder();
     encoder2 = motor2.getEncoder();


 }

 public void Move(double speed){
  motor1.set(speed);
  motor2.set(speed);
 }

 public void Stop(){
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
  if(encoder1.getVelocity() >= RPM - 250 && encoder1.getVelocity() <= RPM + 250){
   return true;
  }
  return false;
 }
 public boolean isAtRPM2(double RPM){
  if(encoder2.getVelocity() >= RPM - 250 && encoder2.getVelocity() <= RPM + 250){
   return true;
  }
  return false;
 }




}
