package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveWithXBox;
import frc.robot.commands.ForwardNote;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShooterAMP;
import frc.robot.commands.ShooterSpeaker;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
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
    private Arm arm;
    private XboxController xboxController;

    private ArmCommand armCommand;
    private boolean shouldBrakeArm;

    @Override
    public void robotInit() {
        DataLogManager.stop();

        this.swerve = SystemFactory.createSwerve();
        this.climb = new Climb();
        this.shooter = new Shooter();
        this.intake = new Intake();
        this.arm = new Arm();

        this.xboxController = new XboxController(0);

        armCommand = new ArmCommand(arm);
        arm.setDefaultCommand(armCommand);

        DriveWithXBox driveWithXBox = new DriveWithXBox(swerve, xboxController);
        swerve.setDefaultCommand(driveWithXBox);

       /*new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new UpAndDown(climb, true));
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new UpAndDown(climb, false));
        */

        new JoystickButton(xboxController, XboxController.Button.kY.value).onTrue(shooterAMP());
        new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(shooterSpeaker());
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new IntakeOut(intake));
        new JoystickButton(xboxController, XboxController.Button.kA.value).onTrue(collectFromFloor());

        new POVButton(xboxController, 0).onTrue(Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_AMP_ANGLE)));
        new POVButton(xboxController, 180).onTrue(Commands.runOnce(() -> armCommand.gentlyDrop()));

        SmartDashboard.putBoolean("ArmDisabledBrake", false);
    }

    @Override
    public void disabledInit() {
        double armPosition = arm.getAngleDegrees();
        shouldBrakeArm = armPosition >= RobotMap.ARM_ANGLE_BEFORE_STOP;

        if (shouldBrakeArm) {
            SmartDashboard.putBoolean("ArmDisabledBrake", true);
            arm.enterBrake();
        }
    }

    @Override
    public void disabledPeriodic() {
        double armPosition = arm.getAngleDegrees();
        if (shouldBrakeArm && armPosition <= RobotMap.ARM_FLOOR_ANGLE + 1) {
            shouldBrakeArm = false;
            SmartDashboard.putBoolean("ArmDisabledBrake", false);

            arm.exitBrake();
        }
    }

    @Override
    public void disabledExit() {
        if (shouldBrakeArm) {
            shouldBrakeArm = false;
            SmartDashboard.putBoolean("ArmDisabledBrake", true);

            arm.exitBrake();
        }
    }

    @Override
    public void teleopInit() {
        armCommand.changeTarget(RobotMap.ARM_DEFAULT_ANGLE);
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
        CommandScheduler.getInstance().run();

    }

    @Override
    public void simulationPeriodic() {

    }

    private Command shooterAMP() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_AMP_ANGLE)),
                Commands.waitUntil(() -> armCommand.didReachTarget()),
                new ParallelCommandGroup(
                        new ShooterAMP(shooter, intake),
                        new ForwardNote(shooter, intake, false)
                ),
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEFAULT_ANGLE))
        );
    }

    private Command shooterSpeaker() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_SPEAKER_ANGLE)),
                Commands.waitUntil(() -> armCommand.didReachTarget()),
                new ParallelCommandGroup(
                        new ShooterSpeaker(shooter, intake),
                        new ForwardNote(shooter, intake, true)
                ),
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEFAULT_ANGLE))
        );
    }

    private Command collectFromFloor() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> armCommand.gentlyDrop()),
                Commands.waitUntil(() -> armCommand.didReachTarget()),
                new IntakeIn(intake),
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEFAULT_ANGLE))
        );
    }
}
