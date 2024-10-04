package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
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

public class Robot extends TimedRobot {

    private Swerve swerve;
    private Climb climb;
    private Shooter shooter;
    private Intake intake;
    private XboxController xboxController;
    private LimelightHelpers limelightHelper;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");

    @Override
    public void robotInit() {
        this.swerve = SystemFactory.createSwerve();
        this.climb = new Climb();
        this.shooter = new Shooter();
        this.intake = new Intake();

        this.xboxController = new XboxController(0);

        DriveWithXBox driveWithXBox = new DriveWithXBox(swerve, xboxController);
        swerve.setDefaultCommand(driveWithXBox);

       /*new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new UpAndDown(climb, true));
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new UpAndDown(climb, false));
        */
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new ShooterAMP(shooter, RobotMap.SHOOTER_SPEED_AMP, intake));
        new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new ShooterSpeaker(shooter, RobotMap.SHOOTER_SPEED_SPEAKER, intake));

        new JoystickButton(xboxController, XboxController.Button.kY.value)
                .onTrue(new IntakeIn(intake));
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new IntakeOut(intake));

        robotPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("Limelight-banana");

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }
    double[] robotPoseTargetSpace;

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {
        ReplanningConfig replanningConfig = new ReplanningConfig(
                false,
                false);
        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(0.5, 0, 0.00007),
                new PIDConstants(0.5, 0, 0.00007),
                4.4169,
                RobotMap.CHASSIS_RADIUS,
                replanningConfig
        );
        PathPlannerPath pathS = PathPlannerPath.fromPathFile("Off-season-check");
        FollowPathHolonomic pathHolonomic = new FollowPathHolonomic(
                pathS,
                swerve::getPose,
                swerve::getSpeeds,
                swerve::drive,
                holonomicPathFollowerConfig,
                () -> {
                    return false;
                },
                swerve);
        pathHolonomic.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        this.swerve.updatePoseEstimator();
        if(LimelightHelpers.getTV("limelight-banana")){             //create a function
            swerve.updatePoseEstimatorByVision(LimelightHelpers.getBotPose2d_wpiBlue("limelight-banana"));
        }
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("Target-Seen",LimelightHelpers.getTV("limelight-banana"));
        SmartDashboard.putNumber("tx", table.getEntry("tx").getDouble(0.0) );
        SmartDashboard.putNumber("ty",table.getEntry("ty").getDouble(0.0));
        SmartDashboard.putNumber("distance2",NetworkTableInstance.getDefault().getTable("limelight-banana").getEntry("botpose").getDoubleArray(new double[6])[2]);
        SmartDashboard.putNumber("distance5",NetworkTableInstance.getDefault().getTable("limelight-banana").getEntry("botpose").getDoubleArray(new double[6])[5]);
        double distance = Math.sqrt(
                Math.pow(robotPoseTargetSpace[0],2)+
                        Math.pow(robotPoseTargetSpace[1],2)+
                        Math.pow(robotPoseTargetSpace[2],2)
        );
        SmartDashboard.putNumber("distancetargetspace", distance);
    }

    @Override
    public void simulationPeriodic() {

    }
}
