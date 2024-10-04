package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Collections;

public class Robot extends TimedRobot {

    private Swerve swerve;
    private Climb climb;
    private Shooter shooter;
    private Intake intake;
    private Arm arm;

    private XboxController xboxController;

    private ArmCommand armCommand;

    @Override
    public void robotInit() {
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

        Pose2d fakeTarget = new Pose2d(0, 2, Rotation2d.fromDegrees(180));
        swerve.getField().getObject("target").setPose(fakeTarget);

        new JoystickButton(xboxController, XboxController.Button.kY.value).onTrue(shooterAMP());
        new JoystickButton(xboxController,XboxController.Button.kB.value).onTrue(shooterSpeaker());
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new IntakeOut(intake));
        new JoystickButton(xboxController, XboxController.Button.kA.value).onTrue(collectFromFloor());


        new JoystickButton(xboxController, XboxController.Button.kStart.value).onTrue(new DeferredCommand(()-> {
            TargetInfo targetInfo = swerve.getTargetInfoFromCurrentPos(fakeTarget);
            SmartDashboard.putNumber("FakeTargetStart", swerve.getHeadingDegrees().getDegrees());
            SmartDashboard.putNumber("FakeTargetAngle", targetInfo.getAngle());
            return new RotateToAngle(swerve, targetInfo.getAngle());
        }, Collections.emptySet()));

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

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
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {

    }

    private Command shooterAMP(){
        return new ParallelCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_AMP_ANGLE)),
                new ShooterAMP(shooter, intake),
                new ForwardNote(shooter, intake, false));
    }

    private Command shooterSpeaker(){
        return new ParallelCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_SPEAKER_ANGLE)),
                new ShooterSpeaker(shooter, intake),
                new ForwardNote(shooter, intake, true));
    }

    private Command collectFromFloor(){
        return new ParallelCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_ANGLE_BEFORE_STOP)),
                new IntakeIn(intake));

    }
}
