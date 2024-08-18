package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
        double driveY = xboxController.getLeftY();
        double driveX = xboxController.getLeftX();
        double rotation = -xboxController.getRightX();
        swerve.drive(driveX,driveY,rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
