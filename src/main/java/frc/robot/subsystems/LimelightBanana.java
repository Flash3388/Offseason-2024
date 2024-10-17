package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
import frc.robot.Vector2D;

import java.util.Optional;


public class LimelightBanana {

    private LimelightHelpers limelightHelper;
    private AprilTagFieldLayout layout;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");
    double[] robotPoseTargetSpace;
    private Swerve swerve;
    private UsbCamera usbCamera1;
    private UsbCamera usbCamera2;

    public LimelightBanana(Swerve swerve){
        table.getEntry("pipeline").setValue(1); //what is the value
        robotPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("Limelight-banana");
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        limelightHelper = new LimelightHelpers();
        this.swerve = swerve;
        this.usbCamera1 = CameraServer.startAutomaticCapture(0);
        this.usbCamera2 = CameraServer.startAutomaticCapture(1);
    }
    public double distanceWithoutVision(){
        double distance = 0.0;
        int id=13;      //if needed is the id target-for example speaker id of blue alliance is 7

        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red){ //red alliance
            id=13;
        }
        //id = (int) LimelightHelpers.getFiducialID("limelight-banana");

        Optional<Pose3d> tagPoseOptional = layout.getTagPose(id);
        if(tagPoseOptional.isEmpty()){                    //not found
            return 0;
        }
        Pose3d tagPose = tagPoseOptional.get(); //if didn't check would crash
        Pose2d robotPose = swerve.getRobotPose();
        distance = Math.sqrt(
                Math.pow(tagPose.getX()-robotPose.getX(),2)
                        +Math.pow(tagPose.getY()-robotPose.getY(),2));

        return distance;
    }
    public boolean changePipeLine(double distance){
        if(distance<=1.5){
            LimelightHelpers.setPipelineIndex("limelight-banana",1);
            return true;
        } else if (distance>1.5&&distance<=3.5) {
            LimelightHelpers.setPipelineIndex("limelight-banana",0);
            return true;
        }
        return false;
    }
    public boolean readyToShoot(){
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return false;
        }

        double aprilTagId = 7; //default is blue alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
            aprilTagId =4;

        Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPoseOptional.isEmpty()) {
            return false;
        }
        Pose3d apriltagPose =apriltagPoseOptional.get();
        Pose2d robotPose =swerve.getRobotPose();
      double distance = Math.sqrt(
                Math.pow(apriltagPose.getX()-robotPose.getX(),2)
                        +Math.pow(apriltagPose.getY()-robotPose.getY(),2));
      return distance<=4.5;
    }
    public double distanceWithVision(){
        double distance = 0.0;
        int id;

        id = (int) LimelightHelpers.getFiducialID("limelight-banana");

        Optional<Pose3d> tagPoseOptional = layout.getTagPose(id);
        if(tagPoseOptional.isEmpty()){                    //not found
            return 0;
        }
        Pose3d tagPose = tagPoseOptional.get(); //if didn't check would crash
        Pose2d robotPose = swerve.getRobotPose();
        distance = Math.sqrt(
                Math.pow(tagPose.getX()-robotPose.getX(),2)
                            +Math.pow(tagPose.getY()-robotPose.getY(),2));
        return distance;
    }
    public double getXAngleToTarget_Speaker() {// for speaker
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double aprilTagId = 7; //default is blue alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
            aprilTagId =4;

        Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPoseOptional.isEmpty()) {
            return 0;
        }

        Pose3d apriltagPose = apriltagPoseOptional.get();
        Pose2d robotPose = swerve.getRobotPose();

        double deltaX = apriltagPose.getX() - robotPose.getX();
        double deltaY = apriltagPose.getY() - robotPose.getY();
        double angleToSpeakerRad= Math.atan2(deltaY,deltaX);
        double angleToSpeakerDeg= Math.toDegrees(angleToSpeakerRad);
        double angleFromRobotToSpeaker = angleToSpeakerDeg - robotPose.getRotation().getDegrees();
        //normalize the angles
        if(angleFromRobotToSpeaker >180) angleFromRobotToSpeaker-=360;
        else if (angleFromRobotToSpeaker <-180) angleFromRobotToSpeaker+=360;

        return angleFromRobotToSpeaker;
    }
    public double getXAngleToTarget_Amp() {// for amp degrees

        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            return 0;
        }

        double aprilTagId = 6; //default is blue alliance
        DriverStation.Alliance alliance = allianceOptional.get();
        if(alliance == DriverStation.Alliance.Red) //if are we red alliance
            aprilTagId = 5;

        Optional<Pose3d> apriltagPoseOptional = layout.getTagPose((int)(aprilTagId)); //position of apriltag
        if (apriltagPoseOptional.isEmpty()) {
            return 0;
        }

        Pose3d apriltagPose = apriltagPoseOptional.get();
        Pose2d robotPose = swerve.getRobotPose();

        double angleFromRobotToSpeaker = 90 - robotPose.getRotation().getDegrees();
        //normalize the angles
        if(angleFromRobotToSpeaker >180) angleFromRobotToSpeaker-=360;
        else if (angleFromRobotToSpeaker <-180) angleFromRobotToSpeaker+=360;

        return angleFromRobotToSpeaker;
    }
    public Pose2d getPose2dBlue(){return LimelightHelpers.getBotPose2d_wpiBlue("limelight-banana");}
    public boolean targetIsSeen(){return LimelightHelpers.getTV("limelight-banana");}
    public void PrintAll(){
        SmartDashboard.putBoolean("Target-Seen",LimelightHelpers.getTV("limelight-banana"));
        SmartDashboard.putNumber("tx", table.getEntry("tx").getDouble(0.0) );
        SmartDashboard.putNumber("ty",table.getEntry("ty").getDouble(0.0));
      //SmartDashboard.putNumber("distance2",NetworkTableInstance.getDefault().getTable("limelight-banana").getEntry("botpose").getDoubleArray(new double[6])[2]); //returns the z meters (height)
    //  SmartDashboard.putNumber("distance5",NetworkTableInstance.getDefault().getTable("limelight-banana").getEntry("botpose").getDoubleArray(new double[6])[5]); // returns the y angle from apriltag
    }
}
