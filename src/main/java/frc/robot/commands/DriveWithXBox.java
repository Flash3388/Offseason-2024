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
        double driveY =   xboxController.getLeftY()  ;
        double driveX = xboxController.getLeftX() ;
        double rotation = xboxController.getRightX() ;
        driveX = Math.abs(driveX) > 0.1 ? driveX * driveX * Math.signum(driveX) * RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE : 0;
        driveY = Math.abs(driveY) > 0.1 ? driveY * driveY * Math.signum(driveY) *  RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE: 0;
        rotation = Math.abs(rotation) > 0.1 ? rotation * Math.signum(rotation) * rotation * RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE : 0;
        swerve.drive(driveX ,driveY,rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
