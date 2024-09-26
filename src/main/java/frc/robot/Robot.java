package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {

    private Arm arm;
    private ArmCommand armCommand;

    private XboxController xbox;

    // for test mode
    private double wantedArmPosition = 0;

    @Override
    public void robotInit() {
        arm = new Arm();

        xbox = new XboxController(0);

        armCommand = new ArmCommand(arm);
        arm.setDefaultCommand(armCommand);
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
        wantedArmPosition = 0;
        SmartDashboard.putNumber("wantedArmPosition", 0);
    }

    @Override
    public void testPeriodic() {
        double position = SmartDashboard.getNumber("wantedArmPosition", 0);
        if(wantedArmPosition != position && position > 0){
            wantedArmPosition = position;
            armCommand.changeTarget(position);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {

    }
}
