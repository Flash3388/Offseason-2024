package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RotateShooter;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
    private Shooter shooter;
    private XboxController xboxController;

    @Override
    public void robotInit() {
        this.shooter = new Shooter();
        this.xboxController = new XboxController(0);
        new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(new RotateShooter(shooter, 0));
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
