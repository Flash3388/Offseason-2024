package frc.robot;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;


public class SystemFactory {


    private SystemFactory() {

    }

    public static Swerve createSwerve() {

        SwerveModule[] swerveModules = new SwerveModule[]{
                createInvertedModule(RobotMap.SWERVE_LF_DRIVE_ID, RobotMap.SWERVE_LF_STEER_ID, 5, RobotMap.SWEARVE_ABSOLUTE_FL_ZERO_ANGLE, "LF"),//LF
                createInvertedModule(RobotMap.SWERVE_RF_DRIVE_ID, RobotMap.SWERVE_RF_STEER_ID, 4, RobotMap.SWEARVE_ABSOLUTE_FR_ZERO_ANGLE, "RF"),//RF
                createInvertedModule(RobotMap.SWERVE_LB_DRIVE_ID, RobotMap.SWERVE_LB_STEER_ID, 6, RobotMap.SWEARVE_ABSOLUTE_RL_ZERO_ANGLE, "LB"),//LB
                createInvertedModule(RobotMap.SWERVE_RB_DRIVE_ID, RobotMap.SWERVE_RB_STEER_ID, 3, RobotMap.SWEARVE_ABSOLUTE_RR_ZERO_ANGLE, "RB"),//RB

        };
        return new Swerve(swerveModules);
    }


    public static SwerveModule createInvertedModule(int drive, int steer, int encoder, double zeroAngle, String identifier) {
        CANSparkMax spark = new CANSparkMax(drive, CANSparkLowLevel.MotorType.kBrushless);
        spark.restoreFactoryDefaults();

        CANSparkMax steerMotor = new CANSparkMax(steer, CANSparkLowLevel.MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();

        CANcoder caNcoder = new CANcoder(encoder);
        caNcoder.getConfigurator().apply(new CANcoderConfiguration());

        return new SwerveModule(spark,
                steerMotor,
                caNcoder,
                zeroAngle,
                identifier);
    }
}
