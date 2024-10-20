package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeOutToShooter extends Command {

    private Intake intake;

    public IntakeOutToShooter(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.in();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
