package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.UpAndDown;
import frc.robot.subsystems.Climb;

public class Robot extends TimedRobot {
    private Climb climb;
    private XboxController xbox;

    @Override
    public void robotInit() {
        xbox=new XboxController(0);
        climb= new Climb();
        new JoystickButton(xbox, XboxController.Button.kA.value)
                .onTrue(new UpAndDown(climb, true));

    new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new UpAndDown(climb,false));}

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
        climb.print();
    }

    @Override
    public void simulationPeriodic() {

    }
}
