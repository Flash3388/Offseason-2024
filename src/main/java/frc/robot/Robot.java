package frc.robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

    private Swerve swerve;
    private Climb climb;
    private Shooter shooter;
    private Intake intake;
    private XboxController xboxController;
    private LimelightBanana limelightBanana;
    private LimelightHelpers limelightHelper;
    private AprilTagFieldLayout layout;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-banana");

    @Override
    public void robotInit() {
        this.swerve = SystemFactory.createSwerve();
        this.limelightBanana = new LimelightBanana(this.swerve);
      //  this.climb = new Climb();
      //  this.shooter = new Shooter();
      //  this.intake = new Intake();
        table.getEntry("pipeline").setValue(1); //what is the value

        this.xboxController = new XboxController(0);

        DriveWithXBox driveWithXBox = new DriveWithXBox(swerve, xboxController);
        swerve.setDefaultCommand(driveWithXBox);

       /*new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new UpAndDown(climb, true));
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new UpAndDown(climb, false));

        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new ShooterAMP(shooter, RobotMap.SHOOTER_SPEED_AMP, intake));
        new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new ShooterSpeaker(shooter, RobotMap.SHOOTER_SPEED_SPEAKER, intake));

        new JoystickButton(xboxController, XboxController.Button.kY.value)
                .onTrue(new IntakeIn(intake));
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new IntakeOut(intake));
*/
        robotPoseTargetSpace = LimelightHelpers.getCameraPose_TargetSpace("Limelight-banana");
        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
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
        int id;
        double distance;  //if needed is the id target-for example speaker id of blue alliance is 7
        /*if(!LimelightHelpers.getTV("limelight-banana")){
             id = (int) LimelightHelpers.getFiducialID("limelight-banana");

            Optional<Pose3d> tagPoseOptional = layout.getTagPose(id);
            if(tagPoseOptional.isEmpty()){                    //not found
                return;
            }
            Pose3d tagPose = tagPoseOptional.get(); //if didn't check would crash
            Pose2d robotPose = swerve.getRobotPose();
             distance = Math.sqrt(
                    Math.pow(tagPose.getX()-robotPose.getX(),2)
                            +Math.pow(tagPose.getY()-robotPose.getY(),2));
            SmartDashboard.putNumber("distance with id", distance);
        }
         */
    }

    @Override
    public void autonomousInit() {
        /*
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

         */
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
        double distance;
        distance = limelightBanana.distanceWithVision();
        SmartDashboard.putNumber("distance with vision", distance);
        distance = limelightBanana.distanceWithoutVision();
        SmartDashboard.putNumber("distance with id", distance);

        //this.limelightBanana.PrintAll();
        //this.swerve.setRobotPoseField(new Pose2d(4.3,6.1,new Rotation2d(0)));
        this.swerve.periodic();
       /* double distance = Math.sqrt(
                Math.pow(robotPoseTargetSpace[0],2)+
                        Math.pow(robotPoseTargetSpace[1],2)+
                        Math.pow(robotPoseTargetSpace[2],2)
        );
        */


        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {

    }
}
