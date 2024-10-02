package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.sql.Time;

public class ShooterAMP extends Command {
    //Are we supposed to delete those comments?
    private final Shooter shooter;
    private final double speed;
    private final Intake intake;
    private final Timer timer;
    //private final Timer fullRunTimer;
    //private boolean timerStarted;

    public ShooterAMP(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.speed = RobotMap.SHOOTER_SPEED_AMP;
        this.intake = intake;
        this.timer = new Timer();
        //this.fullRunTimer = new Timer();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetPid();
        timer.reset();
        timer.start();

        //fullRunTimer.reset();
        //fullRunTimer.start();

        //timerStarted = false;
    }

    @Override
    public void execute() {
        shooter.movePid(speed);

        /*if (!timerStarted && !intake.hasBall()) {
            timerStarted = true;
            SmartDashboard.putNumber("TimeToNoBall", fullRunTimer.get());
            timer.start();
        }

         */
    }

    @Override
    public void end(boolean interrupted) {
        //SmartDashboard.putNumber("TimeToEnd", fullRunTimer.get());
        shooter.stop();
        timer.stop();
        //fullRunTimer.stop();
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
