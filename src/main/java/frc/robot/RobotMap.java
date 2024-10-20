package frc.robot;

public class RobotMap {

    public static final int ARM_MASTER = 16;
    public static final int ARM_FOLLOW = 15;
    public static final double ARM_FLOOR_ANGLE = 15.4;
    public static final double ARM_ANGLE_BEFORE_STOP = 15.7;
    public static final double ARM_CEILING_ANGLE = 123;
    public static final double ARM_AMP_ANGLE = 120;
    public static final double ARM_SPEAKER_ANGLE = 30;
    public static final double ARM_DEFAULT_ANGLE = 60;

    public final static double SHOOTER_RIGHT_KP1 = 0.00000175;
    public final static double SHOOTER_RIGHT_KP2 = 0.00018;
    public final static double SHOOTER_LEFT_KP1 = 0.00000178;
    public final static double SHOOTER_LEFT_KP2 = 0.00018;
    public static final int SHOOTER_MOTOR_LEFT = 14;
    public static final int SHOOTER_MOTOR_RIGHT = 12;
    public static final double SHOOTER_SPEED_AMP = 1000;
    public static final double SHOOTER_SPEED_SPEAKER = 4000;

    public static final int INTAKE_ID_MOTOR = 13;
    public static final int INTAKE_ID_LEFT = 6;
    public static final int INTAKE_ID_RIGHT = 8;
    public static final double ROTATION_FIX_KP =0.002;
    public static final double ROTATION_FIX_KI =0;
    public static final double ROTATION_FIX_KD =0.0001;
    public static final double ROTATION_FIX_KF =0;


    public static final int PIGEON = 9;
    public static final double ATTAINBLE_MAX_SPEED_MPS_SWERVE = 4.5;
    public static final double SWEARVE_ABSOLUTE_FL_ZERO_ANGLE = 2.2;
    public static final double SWEARVE_ABSOLUTE_FR_ZERO_ANGLE = 144;
    public static final double SWEARVE_ABSOLUTE_RL_ZERO_ANGLE = -129.3;
    public static final double SWEARVE_ABSOLUTE_RR_ZERO_ANGLE = 13;
    public static final int SWERVE_LF_DRIVE_ID = 51;
    public static final int SWERVE_RF_DRIVE_ID = 41;
    public static final int SWERVE_LB_DRIVE_ID = 61;
    public static final int SWERVE_RB_DRIVE_ID = 31;


    public static final int SWERVE_LF_STEER_ID = 52;
    public static final int SWERVE_RF_STEER_ID = 42;
    public static final int SWERVE_LB_STEER_ID = 62;
    public static final int SWERVE_RB_STEER_ID = 32;
    public static final double CHASSIS_RADIUS = 0.63;
    public static final double DISTANCE_MODULE_TO_CENTER_CHASSIS_METERS = 0.37;
    public static final double MAX_SPEED_SWERVE = 4.4;

    public static final int CLIMB_MOTOR_ID = 18;
}
