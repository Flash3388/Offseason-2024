package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

public class ArmPid extends Command {
    private final Arm arm;
    private double position;
    private boolean newPosition;
    private static final double TIME_EXTENSION_SEC = 60;   //maybe make it longer
    private double timeLimit;


    public ArmPid(Arm arm){
        this.arm = arm;
        this.position = -1;
        this.newPosition = false;
        this.timeLimit = Timer.getFPGATimestamp() + TIME_EXTENSION_SEC;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.timeLimit = Timer.getFPGATimestamp() + TIME_EXTENSION_SEC;
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

        if(Timer.getFPGATimestamp() > timeLimit){
            changeTarget(RobotMap.ARM_ANGLE_BEFORE_STOP);
        }

        if(didReachTarget() && position == RobotMap.ARM_FLOOR_ANGLE){
            changeTarget(-1);
        }


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
        this.timeLimit = Timer.getFPGATimestamp() + TIME_EXTENSION_SEC;
        if((position < RobotMap.ARM_CEILING_ANGLE && position > RobotMap.ARM_FLOOR_ANGLE) || (position == -1)){
            this.position = position;
        }
        this.newPosition = true;

    }

    public boolean didReachTarget() {
        if (newPosition) return false;

        if(MathUtil.isNear(position, arm.getArmAngle(), 3) && Math.abs(arm.getArmVelocity()) < 5){
            return true;
        }
        return false;
    }
}
