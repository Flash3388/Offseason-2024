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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Collections;
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
        /*new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new IntakeOut(intake));
        new JoystickButton(xboxController, XboxController.Button.kA.value).onTrue(collectFromFloor());
        Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEAFULT_ANGLE));
         */

        new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new IntakeIn(intake));
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .onTrue(new ForwardNote(shooter, intake, true));


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

    double velocity;
    @Override
    public void testInit() {
        velocity = 0;
        SmartDashboard.putNumber("velocity", 0);
        armCommand.changeTarget(60);
    }

    @Override
    public void testPeriodic() {
        double velocity = SmartDashboard.getNumber("velocity", 0);
        if (velocity > 0 && this.velocity != velocity) {
            shooter.movePid(velocity);
            this.velocity = velocity;
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {

    }

    private Command shooterAMP(){
        return new SequentialCommandGroup(
         new ParallelCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_AMP_ANGLE)),
                new ShooterAMP(shooter, intake),
                new ForwardNote(shooter, intake, false)),
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEAFULT_ANGLE))

        );

    }

    private Command shooterSpeaker(){
        return new SequentialCommandGroup(
         new ParallelCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_SPEAKER_ANGLE)),
                new ShooterSpeaker(shooter, intake),
                new ForwardNote(shooter, intake, true)),
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEAFULT_ANGLE))


        );

    }

    private Command collectFromFloor(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_ANGLE_BEFORE_STOP)),
                        new IntakeIn(intake)),
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEAFULT_ANGLE))
        );
    }
}
