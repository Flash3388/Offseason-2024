package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class CheckPID extends Command {
    private Shooter shooter;
    private double speed;

    public CheckPID(Shooter shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;
        addRequirements(shooter);
        SmartDashboard.putNumber("speedShooter", 0);
    }

    @Override
    public void initialize() {
        speed = SmartDashboard.getNumber("speedShooter", 0);
        shooter.resetPid();
    }

    @Override
    public void execute() {
        shooter.movePid(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
