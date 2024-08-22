package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;


public class SystemFactory {


    public SystemFactory(){

    }

    public static Swerve createSwerve(){

        SwerveModule[] swerveModules = new SwerveModule[] {
                createInvertedModule(RobotMap.SWERVE_LF_DRIVE_ID, RobotMap.SWERVE_LF_STEER_ID, 5,RobotMap.SWEARVE_ABSOLUTE_FL_ZERO_ANGLE, "LF"),//LF
                createInvertedModule(RobotMap.SWERVE_RF_DRIVE_ID, RobotMap.SWERVE_RF_STEER_ID, 4,RobotMap.SWEARVE_ABSOLUTE_FR_ZERO_ANGLE, "RF"),//RF
                createInvertedModule(RobotMap.SWERVE_LB_DRIVE_ID, RobotMap.SWERVE_LB_STEER_ID, 6,RobotMap.SWEARVE_ABSOLUTE_RL_ZERO_ANGLE, "LB"),//LB
                createInvertedModule(RobotMap.SWERVE_RB_DRIVE_ID, RobotMap.SWERVE_RB_STEER_ID, 3,RobotMap.SWEARVE_ABSOLUTE_RR_ZERO_ANGLE, "RB"),//RB

        };
        return new Swerve(swerveModules);
    }


    public static SwerveModule createInvertedModule(int drive, int steer, int encoder, double zeroAngle, String identifier) {
        CANSparkMax spark = new CANSparkMax(drive, CANSparkLowLevel.MotorType.kBrushless);
        spark.restoreFactoryDefaults();
        spark.setInverted(true);

        CANSparkMax steerMotor = new CANSparkMax(steer, CANSparkLowLevel.MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();

        return new SwerveModule(spark,
                steerMotor,
                new CANCoder(encoder),
                zeroAngle,
                identifier);
    }
}
