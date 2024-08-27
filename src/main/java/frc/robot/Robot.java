package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmMoveDown;
import frc.robot.commands.ArmMoveUp;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
    private Arm arm;

    private XboxController xbox;

    @Override
    public void robotInit() {
        arm = new Arm();

        xbox = new XboxController(0);

        new JoystickButton(xbox, XboxController.Button.kX.value).whileTrue(new ArmMoveDown(arm));
        new JoystickButton(xbox, XboxController.Button.kY.value).whileTrue(new ArmMoveUp(arm));
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
        arm.print();
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
