package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class UpAndDown extends Command {
    private Climb climb;
    private boolean direction;

    public UpAndDown(Climb climb, boolean direction) {
        this.climb = climb;
        this.direction= direction;
        addRequirements(climb);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climb.rotateMotor(direction);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        if (direction){
            return climb.getf();
        }
        else {
            return climb.getr();
        }
    }
}
