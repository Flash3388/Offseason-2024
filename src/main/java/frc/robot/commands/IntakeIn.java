package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeIn extends Command {

    private Intake intake;

    public IntakeIn(Intake intake) {
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

        if (!interrupted) {
            DriverStation.reportWarning("has ball man!", false);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.hasBall();
    }
}
