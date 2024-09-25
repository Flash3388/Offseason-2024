package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmPid;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {
    private Arm arm;
    private ArmPid armPid;

    private XboxController xbox;

    @Override
    public void robotInit() {
        arm = new Arm();
        armPid = new ArmPid(arm);
        arm.setDefaultCommand(armPid);

        xbox = new XboxController(0);

        armPid.changeTarget(60);

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

    double pos = 0;
    @Override
    public void autonomousInit() {
        pos = 0;
        SmartDashboard.putNumber("position", 0);
    }
    @Override
    public void autonomousPeriodic() {
        double position= SmartDashboard.getNumber("position", 0);
        if(pos != position && position > 0){
            pos = position;
            armPid.changeTarget(position);
        }
        SmartDashboard.putBoolean("didReachTarget", armPid.didReachTarget());
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
