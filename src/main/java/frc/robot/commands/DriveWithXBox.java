package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;

public class DriveWithXBox extends Command {
    private Swerve swerve;
    private XboxController xboxController;
    public DriveWithXBox(Swerve swerve, XboxController xboxController){
    this.swerve = swerve;
    this.xboxController = xboxController;
    addRequirements(swerve);

    }

    @Override
    public void execute() {
        double driveY = xboxController.getLeftY() * RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE;
        double driveX = xboxController.getLeftX() * RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE;
        double rotation = -xboxController.getRightX() *RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE;
        swerve.drive(driveX ,driveY,rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
