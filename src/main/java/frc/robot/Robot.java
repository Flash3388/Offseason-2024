package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RotateShooter;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
    private Shooter shooter;
    private XboxController xboxController;
    private Intake intake;

    @Override
    public void robotInit() {
        intake = new Intake();
        xboxController = new XboxController(0);
        new JoystickButton(xboxController, XboxController.Button.kY.value).onTrue(new IntakeIn(intake));
        new JoystickButton(xboxController, XboxController.Button.kX.value).whileTrue(new IntakeOut(intake));

        this.shooter = new Shooter();
        new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(new RotateShooter(shooter, 0, intake));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        shooter.stop();
    }

    @Override
    public void teleopPeriodic() {
        intake.print();

    }

    @Override
    public void autonomousInit() {

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
        shooter.print();
    }

    @Override
    public void simulationPeriodic() {

    }
}
