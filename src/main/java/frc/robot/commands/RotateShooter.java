package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RotateShooter extends Command {
    private Shooter shooter;
    private double speed;
    public RotateShooter(Shooter shooter,double speed){
        this.shooter=shooter;
        this.speed=speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
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
        return shooter.isAtRPM1(speed) && shooter.isAtRPM2(speed);
    }

}
