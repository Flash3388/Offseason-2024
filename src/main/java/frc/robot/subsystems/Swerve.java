package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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

import static edu.wpi.first.units.Units.Meters;

public class Swerve extends SubsystemBase {


    private final Pigeon2 pigeon;
    private final SwerveDriveOdometry odometry;
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SysIdRoutine sysIdRoutine;
    private final Field2d field;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pose2d robotPose;

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
        odometry = new SwerveDriveOdometry(kinematics, getHeadingDegrees(), getModulesPosition());
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        robotPose = new Pose2d();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kinematics,getHeadingDegrees(),getModulesPosition(),robotPose);
        swerveDrivePoseEstimator.addVisionMeasurement(robotPose,Timer.getFPGATimestamp());
    }

    public Rotation2d getHeadingDegrees(){
        double degrees = pigeon.getRotation2d().getDegrees();
        degrees %= 360;
        if(degrees < 0){
            degrees += 360;
        }

        return Rotation2d.fromDegrees(degrees);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
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

    public double getDistancePassedMetersLeft(){return ((-swerveModules[0].getDistancePassedMeters()+(-swerveModules[2].getDistancePassedMeters())/2));}
    public double getDistancePassedMetersLeft2(){return (-swerveModules[0].getDistancePassedMeters());}
    public double getDistancePassedMetersRight(){return ((-swerveModules[1].getDistancePassedMeters()+(-swerveModules[3].getDistancePassedMeters())/2));}
    public double getDistancePassedMetersRight2(){return (-swerveModules[1].getDistancePassedMeters());}
    public void updatePoseEstimatorByVision(Pose2d robotPose){
        this.robotPose=this.swerveDrivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(),getHeadingDegrees(),getModulesPosition());
            setRobotPoseField(this.robotPose);
    }
    public void updatePoseEstimator(){
      this.robotPose=this.swerveDrivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(),getHeadingDegrees(),getModulesPosition());
        setRobotPoseField(robotPose);
    }
    public void setRobotPoseField(Pose2d robotPoseField){field.setRobotPose(robotPoseField);}
    public Pose2d getRobotPose(){return robotPose;}

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void drive(double speedX, double speedY, double rotation) {
        SmartDashboard.putNumber("SwerveCommandX", speedX);
        SmartDashboard.putNumber("SwerveCommandY", speedY);
        SmartDashboard.putNumber("SwerveCommandRot", rotation);

        SwerveModuleState[] swerveModuleStates;
        swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(speedY, speedX, rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE);
        setDesiredState(swerveModuleStates);
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates;
        swerveModuleStates = kinematics.toSwerveModuleStates(speeds);
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

    @Override
    public void periodic() {
        Rotation2d rotation = getHeadingDegrees();
        SmartDashboard.putNumber("SwerveHeading", rotation.getDegrees());
        SmartDashboard.putNumber("HeadingRaw", pigeon.getRotation2d().getDegrees());

        for (int i = 0; i < 4; i++) {
            swerveModules[i].periodic();
        }

        //odometry.update(rotation, getModulesPosition());
        //setRobotPoseField(odometry.getPoseMeters());
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
