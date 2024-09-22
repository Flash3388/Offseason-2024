package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmPid extends Command {
    private final Arm arm;
    private double position;
    private boolean newPosition;

    public ArmPid(Arm arm){
        this.arm = arm;
        this.position = -1;
        this.newPosition = false;

        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (newPosition) {
            newPosition = false;

            if (position > 0) {
                // todo: maybe reset iaccum
                arm.movePid(position);
            } else {
                arm.stop();
            }
        }

        // todo: timer to stop holding arm
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void changeTarget(double position) {
        this.position = position;
        this.newPosition = true;

        // todo: have range limit to position
    }

    public boolean didReachTarget() {
        if (newPosition) return false;

        if(MathUtil.isNear(position, arm.getArmAngle(), 3) && Math.abs(arm.getArmVelocity()) < 5){
            return true;
        }
        return false;
    }
}
