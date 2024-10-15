package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-banana";

    private final Swerve swerve;
    private final NetworkTableEntry hasTargetEntry;
    private final NetworkTableEntry botPoseEntry;

    public Vision(Swerve swerve) {
        this.swerve = swerve;

        NetworkTable rootTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
        hasTargetEntry = rootTable.getEntry("tv");
        botPoseEntry = rootTable.getEntry("botpose_wpiblue");
    }

    @Override
    public void periodic() {
        boolean hasTarget = hasTargetEntry.getBoolean(false);
        SmartDashboard.putBoolean("VisionHasTarget", hasTarget);

        if (!hasTarget) {
            return;
        }

        NetworkTableValue botPoseValue = botPoseEntry.getValue();
        long timestamp = botPoseValue.getTime();
        double[] botPoseArr = botPoseValue.getDoubleArray();
        Pose2d botPose = poseFromArr(botPoseArr).toPose2d();
        double latency = botPoseArr[6];

        double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);
        swerve.updatePoseWithVision(botPose, adjustedTimestamp);

        SmartDashboard.putNumber("VisionPoseX", botPose.getX());
        SmartDashboard.putNumber("VisionPoseY", botPose.getY());
        SmartDashboard.putNumber("VisionPoseAngle", botPose.getRotation().getDegrees());
        SmartDashboard.putNumber("VisionPoseTimestamp", adjustedTimestamp);
    }

    private static Pose3d poseFromArr(double[] arr) {
        assert arr.length >= 6;
        return new Pose3d(
                new Translation3d(arr[0], arr[1], arr[2]),
                new Rotation3d(
                        Math.toRadians(arr[3]),
                        Math.toRadians(arr[4]),
                        Math.toRadians(arr[5])
                )
        );
    }
}
