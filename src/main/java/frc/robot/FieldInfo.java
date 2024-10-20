package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

public class FieldInfo {

    // field coordinate system
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
    // (0, 0) at blue alliance bottom corner
    // 0 heading towards red alliance wall

    // field elements drawings
    // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings-CRESCENDOSpecific.pdf
    // page 120 = speaker 1

    // field layout drawings
    // https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf

    // manual arena section
    // http://firstfrc.blob.core.windows.net/frc2024/Manual/Sections/2024GameManual-05ARENA.pdf

    // speaker opening height = 1.98m -> 2.11m
    // Blue AprilTag = 7
    // Red AprilTag = 4

    public static Pose2d getOurSpeakerPose() {
        // speaker is at alliance wall, so x = 0 or field length

        if (isRedAlliance()) {
            return new Pose2d(16.541, 5.54, Rotation2d.fromDegrees(180));
        } else {
            return new Pose2d(0, 5.54, Rotation2d.fromDegrees(0));
        }
    }

    private static boolean isRedAlliance() {
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        if (allianceOptional.isEmpty()) {
            // alliance is unknown. Perhaps not connect to DriverStation at the moment.
            // use RED as default
            return true;
        }

        DriverStation.Alliance alliance = allianceOptional.get();
        return alliance == DriverStation.Alliance.Red;
    }
}
