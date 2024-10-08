package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Optional;

public class LimelightBanana {

    private LimelightHelpers limelightHelper;
    private AprilTagFieldLayout layout;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");
    double[] robotPoseTargetSpace;

    public LimelightBanana(){
        table.getEntry("pipeline").setValue(1); //what is the value
        robotPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("Limelight-banana");
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        limelightHelper = new LimelightHelpers();
    }
    public double distanceWithId(Swerve swerve){
        double distance = 0.0;
        int id;       //if needed is the id target-for example speaker id of blue alliance is 7

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
    public boolean targetIsSeen(){return LimelightHelpers.getTV("limelight-banana");}
    public void PrintAll(){
        SmartDashboard.putBoolean("Target-Seen",LimelightHelpers.getTV("limelight-banana"));
        SmartDashboard.putNumber("tx", table.getEntry("tx").getDouble(0.0) );
        SmartDashboard.putNumber("ty",table.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("distance2",NetworkTableInstance.getDefault().getTable("limelight-banana").getEntry("botpose").getDoubleArray(new double[6])[2]);
        SmartDashboard.putNumber("distance5",NetworkTableInstance.getDefault().getTable("limelight-banana").getEntry("botpose").getDoubleArray(new double[6])[5]);
    }
}
