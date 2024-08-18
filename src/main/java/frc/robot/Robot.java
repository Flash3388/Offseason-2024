package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveWithXBox;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
    Swerve swerve;
    XboxController xboxController;
    @Override
    public void robotInit() {
        swerve = SystemFactory.createSwerve();
        xboxController = new XboxController(0);
        DriveWithXBox driveWithXBox = new DriveWithXBox(swerve,xboxController);
        swerve.setDefaultCommand(driveWithXBox);
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
