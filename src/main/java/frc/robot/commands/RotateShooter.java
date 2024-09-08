package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RotateShooter extends Command {
    private Shooter shooter;
    private double speed;
    public RotateShooter(Shooter shooter,double speed){
        this.shooter=shooter;
        this.speed=speed;
        addRequirements(shooter);
        SmartDashboard.putNumber("speedShooter", 0);
    }

    @Override
    public void initialize() {
        speed= SmartDashboard.getNumber("speedShooter", 0);
        shooter.resetPid();
    }

    @Override
    public void execute() {
        shooter.movePid(speed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return shooter.isAtRPM1(speed) && shooter.isAtRPM2(speed);
    }

}
