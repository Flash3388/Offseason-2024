package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShooterSpeaker extends Command {
    private Shooter shooter;
    private double speed;
    private Intake intake;
    private Timer timer;

    public ShooterSpeaker(Shooter shooter, Intake intake, double speed) {
        this.shooter = shooter;
        //this.speed = RobotMap.SHOOTER_SPEED_SPEAKER;
        this.speed = speed;
        this.intake = intake;
        this.timer = new Timer();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetPid();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        shooter.movePid(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (intake.hasBall()) {
            timer.restart();
            return false;
        }
        else{
            if(timer.hasElapsed(1)){
                return true;
            }
            return false;
        }
    }
}
