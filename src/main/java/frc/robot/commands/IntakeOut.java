package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeOut extends Command {

    private Intake intake;

    public IntakeOut(){
        this.intake = new Intake();

        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.In();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        if((!(intake.getLeft())) && (!(intake.getRight()))){
            return true;
        }
        return false;
    }
}
