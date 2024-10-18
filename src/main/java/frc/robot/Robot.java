package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;

import java.util.Collections;
import java.util.Set;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveWithXBox;
import frc.robot.commands.ForwardNote;
import frc.robot.commands.IntakeIn;
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

        Pose2d fakeTarget = new Pose2d(5.75, 0, Rotation2d.fromDegrees(180));
        swerve.getField().getObject("target").setPose(fakeTarget);

        new POVButton(xboxController, 0)
                .onTrue(new UpAndDown(climb, true));
        new POVButton(xboxController, 180)
                .onTrue(new UpAndDown(climb, false));

        new JoystickButton(xboxController, XboxController.Button.kY.value)
                .onTrue(shooterAMP());
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(shooterSpeaker());
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new IntakeOut(intake));
        new JoystickButton(xboxController,XboxController.Button.kA.value)
                .onTrue(collectFromFloor());

        new POVButton(xboxController, 90)
                .onTrue(new IntakeIn(intake));
        new POVButton(xboxController, 270)
                .whileTrue(new IntakeOutToShooter(intake));
        new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
                .onTrue(Commands.runOnce(() -> armCommand.gentlyDrop()));
        new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value)
                .onTrue(Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_CLIMB_ANGLE)));


        SmartDashboard.putNumber("Distance", 0);
        new JoystickButton(xboxController, XboxController.Button.kStart.value).onTrue(new DeferredCommand(()-> {
            double distance = SmartDashboard.getNumber("Distance", 0);
            double angle = arm.calculateFiringAngleDegrees(distance);
            if (angle < 0) {
                return null;
            }

            SmartDashboard.putNumber("FakeTargetAngleToHit", angle);

            return new SequentialCommandGroup(
                    Commands.runOnce(() -> armCommand.changeTarget(angle)),
                    Commands.waitUntil(() -> armCommand.didReachTarget()),
                    new ParallelCommandGroup(
                            new ForwardNote(shooter, intake, RobotMap.SHOOTER_SPEED_SPEAKER),
                            new ShooterSpeakerSpeed(shooter, intake, RobotMap.SHOOTER_SPEED_SPEAKER)
                    )

            );
        }, Set.of(shooter, swerve, intake)));

        /*new JoystickButton(xboxController, XboxController.Button.kStart.value).onTrue(new DeferredCommand(()-> {
            TargetInfo targetInfo = swerve.getTargetInfoFromCurrentPos(fakeTarget);
            double angle = arm.calculateFiringAngleDegrees(targetInfo.getDistance());

            SmartDashboard.putNumber("FakeTargetStart", swerve.getHeadingDegrees().getDegrees());
            SmartDashboard.putNumber("FakeTargetAngle", targetInfo.getAngle());
            SmartDashboard.putNumber("FakeTargetAngleToHit", angle);

            return new SequentialCommandGroup(
                    new RotateToAngle(swerve, targetInfo.getAngle()),
                    Commands.runOnce(() -> armCommand.changeTarget(angle)),
                    Commands.waitUntil(() -> armCommand.didReachTarget()),
                    new ParallelCommandGroup(
                            new ForwardNote(shooter, intake, RobotMap.SHOOTER_SPEED_SPEAKER),
                            new ShooterSpeakerSpeed(shooter, intake, RobotMap.SHOOTER_SPEED_SPEAKER)
                    )

            );
        }, Collections.emptySet()));*/
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
        this.velocity = 0;
        this.angle = 0;
        armCommand.changeTarget(RobotMap.ARM_DEFAULT_ANGLE);
        shooter.stop();
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
    double angle;
    @Override
    public void testInit() {
        armCommand.stopHolding();
        velocity = 0;
        angle = 0;
        shooter.stop();
        SmartDashboard.putNumber("velocity", 0);
        SmartDashboard.putNumber("angle", 0 );
    }

    @Override
    public void testPeriodic() {
        double velocity = SmartDashboard.getNumber("velocity", 0);
        double angle = SmartDashboard.getNumber("angle", 0);
        if (velocity >= 0 && this.velocity != velocity) {
            shooter.movePid(velocity);
            this.velocity = velocity;
        }
        if(angle > 0 && this.angle != angle){
            armCommand.changeTarget(angle);
            this.angle = angle;
        }
    }

    PowerDistribution pdh = new PowerDistribution();
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("PDH Port", pdh.getCurrent(6));

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
                        new ForwardNote(shooter, intake, RobotMap.SHOOTER_SPEED_AMP)
                ),
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_DEFAULT_ANGLE))
        );
    }

    private Command shooterSpeaker() {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> armCommand.changeTarget(RobotMap.ARM_SPEAKER_ANGLE)),
                Commands.waitUntil(() -> armCommand.didReachTarget()),
                new ParallelCommandGroup(
                        new ShooterSpeaker(shooter, intake, RobotMap.SHOOTER_SPEED_SPEAKER),
                        new ForwardNote(shooter, intake, RobotMap.SHOOTER_SPEED_SPEAKER)
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
