package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ForwardNote extends Command {

    private Shooter shooter;
    private Intake intake;
    private double speed;
    private Timer timer;

    public ForwardNote(Shooter shooter, Intake intake, double speed) {
        this.shooter = shooter;
        this.intake = intake;
        this.speed = speed;
        this.timer = new Timer();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (shooter.isAtRangePID(speed)) {
            intake.in();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
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
