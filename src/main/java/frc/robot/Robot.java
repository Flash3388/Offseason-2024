package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DriveWithXBox;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShooterAMP;
import frc.robot.commands.ShooterSpeaker;
import frc.robot.subsystems.*;

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
        new JoystickButton(xboxController, XboxController.Button.kB.value)
                .onTrue(new ShooterAMP(shooter, RobotMap.SHOOTER_SPEED_AMP, intake));
        new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new ShooterSpeaker(shooter, RobotMap.SHOOTER_SPEED_SPEAKER, intake));

        new JoystickButton(xboxController, XboxController.Button.kY.value)
                .onTrue(new IntakeIn(intake));
        new JoystickButton(xboxController, XboxController.Button.kX.value)
                .whileTrue(new IntakeOut(intake));
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

}
