package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {

    private static final double MAX_HOLD_ARM_TIME_SEC = 60;
    private static final double POSITION_DROP = -1;

    private final Arm arm;
    private final Timer limitTimer;

    private double position;
    private double newPosition;
    private boolean hasNewPosition;
    private boolean isInTarget;

    public ArmCommand(Arm arm){
        this.arm = arm;
        this.limitTimer = new Timer();

        this.position = POSITION_DROP;
        this.newPosition = POSITION_DROP;
        this.hasNewPosition = false;
        this.isInTarget = false;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        isInTarget = false;
        hasNewPosition = true;
    }

    @Override
    public void execute() {
        if (hasNewPosition) {
            hasNewPosition = false;
            isInTarget = false;

            position = newPosition;
            SmartDashboard.putNumber("ArmCommandTarget", position);

            if (position > 0) {
                // todo: maybe reset iaccum
                arm.setMoveToPosition(position);
                limitTimer.restart();

                SmartDashboard.putBoolean("ArmCommandControl", true);
            } else {
                arm.stop();
                limitTimer.stop();

                SmartDashboard.putBoolean("ArmCommandControl", false);
            }
        }

        isInTarget = position > 0 ?
                        arm.didReachTarget(position) :
                        arm.isAtFloor();
        SmartDashboard.putBoolean("ArmCommandInTarget", isInTarget);

        if (position <= 0) {
            // nothing more to do if we are not moving
            return;
        }

        if (limitTimer.hasElapsed(MAX_HOLD_ARM_TIME_SEC)) {
            // todo: why not floor?
            gentlyDrop();
        }

        if (isInTarget && MathUtil.isNear(position, RobotMap.ARM_FLOOR_ANGLE, 1)) {
            // isNear because double equality is complex
            stopHolding();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    public void changeTarget(double position) {
        if (position == POSITION_DROP || (position < RobotMap.ARM_CEILING_ANGLE && position > RobotMap.ARM_FLOOR_ANGLE)) {
            this.newPosition = position;
            this.hasNewPosition = true;
        } else {
            DriverStation.reportWarning("Request ArmCommand new position which is not valid", true);
        }
    }

    public void gentlyDrop() {
        changeTarget(RobotMap.ARM_ANGLE_BEFORE_STOP);
    }

    public void stopHolding() {
        // todo: dropping is dangerous!!!! we shouldn't normally do this
        //  instead we should move slowly to the floor and then release
        changeTarget(POSITION_DROP);
    }

    public boolean didReachTarget() {
        if (hasNewPosition) {
            return false;
        }

        return isInTarget;
    }
}
