package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;
import frc.robot.TargetInfo;

import static edu.wpi.first.units.Units.Meters;

public class Swerve extends SubsystemBase {


    private final Pigeon2 pigeon;
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SysIdRoutine sysIdRoutine;
    private final Field2d field;
    private final PIDController rotationPID;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    public Swerve(SwerveModule[] swerveModules) {
        sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::volatageDrive, this::sysidLog, this));
        this.swerveModules = swerveModules;
        double distance = RobotMap.DISTANCE_MODULE_TO_CENTER_CHASSIS_METERS;
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(distance, distance),
                new Translation2d(distance, -distance),
                new Translation2d(-distance, distance),
                new Translation2d(-distance, -distance)
        );
        pigeon = new Pigeon2(RobotMap.PIGEON);
        pigeon.setYaw(0);

        rotationPID = new PIDController(RobotMap.ROTATION_FIX_KP,RobotMap.ROTATION_FIX_KI,RobotMap.ROTATION_FIX_KD);

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getHeadingDegrees(),
                getModulesPosition(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogActivePathCallback((poses)-> {
            field.getObject("path").setPoses(poses);
        } );
    }

    public Field2d getField() {
        return field;
    }

    public Rotation2d getHeadingDegrees(){
        double degrees = pigeon.getRotation2d().getDegrees();
        degrees %= 360;
        if(degrees < 0){
            degrees += 360;
        }

        return Rotation2d.fromDegrees(degrees);
    }

    public SwerveModulePosition[] getModulesPosition() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = swerveModules[i].getModulePosition();
        }

        return swerveModulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = swerveModules[i].getModuleStates();
        }

        return swerveModuleStates;
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(new SwerveDriveKinematics.SwerveDriveWheelStates(getModuleStates()));
    }

    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void updatePoseEstimatorByVision(Pose2d estimatedRobotPose, double timestamp) {
        swerveDrivePoseEstimator.addVisionMeasurement(estimatedRobotPose, timestamp);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void drive(double speedX, double speedY, double rotation) {
        drive(new ChassisSpeeds(speedY, speedX, rotation));
    }

    public void drive(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("SwerveCommandX", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("SwerveCommandY", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("SwerveCommandRot", speeds.omegaRadiansPerSecond);
        if(speeds.omegaRadiansPerSecond ==0){
            double currentHeading = getHeadingDegrees().getDegrees();
            if(getHeadingDegrees().getDegrees() > currentHeading+1.5 || getHeadingDegrees().getDegrees() < currentHeading -1.5){
                double newRot = rotationPID.calculate(getHeadingDegrees().getDegrees(),currentHeading);
                speeds = new ChassisSpeeds(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,newRot);
            }
        }

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE);
        setDesiredState(swerveModuleStates);
    }

    public void volatageDrive(Measure<Voltage> voltage) {
        double volts = voltage.in(Units.Volts);
        double output = volts / RobotController.getBatteryVoltage();

        move(output, 0);
    }

    public void setDesiredState(SwerveModuleState[] swerveModuleStates) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void move(double drive, double rotation) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].move(drive, rotation);
        }
    }

    public void setDriveVelocity(double velocityMps) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setDriveVelocity(velocityMps);
        }
    }

    public void setSteerPosition(double positionDegrees) {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setSteerPosition(positionDegrees);
        }
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].stop();
        }
    }

    public TargetInfo getTargetInfoFromCurrentPos(Pose2d other) {
        Pose2d pose = getPose();
        return getTargetInfo(pose, other);
    }

    public TargetInfo getTargetInfo(Pose2d robot, Pose2d other) {
        double x = robot.getX() - other.getX();
        double y = robot.getY() - other.getY();

        double distance = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        double angle = Math.toDegrees(Math.atan2(y, x));
        angle += 180;

        return new TargetInfo(distance, angle);
    }

    public void resetOdometeryToStart() {
        swerveDrivePoseEstimator.resetPosition(
                getHeadingDegrees(),
                getModulesPosition(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))
        );
    }

    @Override
    public void periodic() {
        Rotation2d rotation = getHeadingDegrees();
        SmartDashboard.putNumber("SwerveHeading", rotation.getDegrees());
        SmartDashboard.putNumber("HeadingRaw", pigeon.getRotation2d().getDegrees());

        for (int i = 0; i < 4; i++) {
            swerveModules[i].periodic();
        }

        Pose2d robotPose = swerveDrivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                getHeadingDegrees(),
                getModulesPosition());
        field.setRobotPose(robotPose);
    }

    private void sysidLog(SysIdRoutineLog log) {
        SwerveModule frontLeft = swerveModules[0];
        SwerveModule frontRight = swerveModules[1];

        log.motor("drive-left")
                .voltage(frontLeft.getOutputVoltage())
                .linearPosition(Meters.of(frontLeft.getPositionMeters()))
                .linearVelocity(frontLeft.getLinearVelocity());

        log.motor("drive-right")
                .voltage(frontRight.getOutputVoltage())
                .linearPosition(Meters.of(frontRight.getPositionMeters()))
                .linearVelocity(frontRight.getLinearVelocity());
    }
}
