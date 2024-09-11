package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RotateShooter extends Command {
    private Shooter shooter;
    private double speed;
    private Intake intake;
    public RotateShooter(Shooter shooter, Intake intake){
        this.shooter=shooter;
        this.intake = intake;
        addRequirements(shooter, intake);
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
        if(shooter.isAtRangePID1() && shooter.isAtRangePID2()){
            intake.in();
        }
    }

    @Override
    public void end(boolean interrupted) {
     shooter.stop();
     intake.stop();
    }

    @Override
    public boolean isFinished() {
        if(!intake.checkout()){
            return true;
        }
        return false;
    }

}
