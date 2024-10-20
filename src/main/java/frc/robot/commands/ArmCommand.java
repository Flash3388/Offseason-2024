package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {

    private static final double MAX_HOLD_ARM_TIME_SEC = 200;
    private static final double POSITION_DROP = -1;
    private static final double MAX_VELOCITY_RPM = 1000;
    private static final double MAX_ACCELERATION_RPM_S = 200;
    private static final double ARM_STUCK_TIME_SEC = 5;

    private final Arm arm;
    private final Timer limitTimer;
    private final TrapezoidProfile.Constraints motionProfileConstraints;
    private final Timer armStuckTimer;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;
    private boolean motionProfileFinished;

    private double position;
    private double newPosition;
    private boolean hasNewPosition;
    private boolean isInTarget;
    private double lastIterationPosition;
    private boolean armStuckTimerRunning;
    private boolean armKnownStuck;

    public ArmCommand(Arm arm) {
        this.arm = arm;
        this.limitTimer = new Timer();
        this.motionProfileConstraints = new TrapezoidProfile.Constraints(MAX_VELOCITY_RPM, MAX_ACCELERATION_RPM_S);
        this.armStuckTimer = new Timer();

        this.position = POSITION_DROP;
        this.newPosition = POSITION_DROP;
        this.hasNewPosition = false;
        this.isInTarget = false;
        this.lastIterationPosition = 0;
        this.armStuckTimerRunning = false;
        this.armKnownStuck = false;

        this.motionProfileGoal = new TrapezoidProfile.State();
        this.motionProfileSetPoint = new TrapezoidProfile.State();
        this.motionProfileFinished = false;

        SmartDashboard.putNumber("ArmCommandTarget", position);
        SmartDashboard.putBoolean("ArmCommandControl", false);
        SmartDashboard.putBoolean("ArmCommandInTarget", true);
        SmartDashboard.putNumber("ArmCommandTimeToLimit", -1);
        SmartDashboard.putNumber("ArmCommandProfileSP", -1);
        SmartDashboard.putBoolean("ArmCommandProfileFinished", false);
        SmartDashboard.putBoolean("ArmCommandStuck", false);

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
            SmartDashboard.putNumber("ArmCommandTimeToLimit", -1);
            SmartDashboard.putBoolean("ArmCommandProfileFinished", false);
            SmartDashboard.putBoolean("ArmCommandStuck", false);

            lastIterationPosition = 0;
            armStuckTimerRunning = false;
            armKnownStuck = false;
            armStuckTimer.stop();

            if (position > 0) {
                motionProfile = new TrapezoidProfile(motionProfileConstraints);
                motionProfileGoal = new TrapezoidProfile.State(position, 0);
                motionProfileSetPoint = new TrapezoidProfile.State(arm.getAngleDegrees(), 0);
                motionProfileFinished = false;

                limitTimer.restart();

                SmartDashboard.putBoolean("ArmCommandControl", true);
            } else {
                arm.stop();
                limitTimer.stop();

                motionProfileGoal = null;

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

        if (!motionProfileFinished) {
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
            SmartDashboard.putNumber("ArmCommandProfileSP", motionProfileSetPoint.position);
            arm.setMoveToPosition(motionProfileSetPoint.position);

            if (isInTarget) {
                SmartDashboard.putBoolean("ArmCommandProfileFinished", true);
                motionProfileFinished = true;
            }
        } else  {
            arm.setMoveToPosition(position);
        }

        double currentArmPosition = arm.getAngleDegrees();
        if (!isInTarget && MathUtil.isNear(lastIterationPosition, arm.getAngleDegrees(), 2)) {
            if (!armKnownStuck) {
                if (!armStuckTimerRunning) {
                    armStuckTimerRunning = true;
                    armStuckTimer.restart();
                }

                if (armStuckTimer.hasElapsed(ARM_STUCK_TIME_SEC)) {
                    armKnownStuck = true;
                    SmartDashboard.putBoolean("ArmCommandStuck", true);
                    DriverStation.reportError("Arm is stuck and is no longer moving", false);
                }
            }
        } else if (armKnownStuck || armStuckTimerRunning) {
            armKnownStuck = false;
            armStuckTimerRunning = false;
            armStuckTimer.stop();
            SmartDashboard.putBoolean("ArmCommandStuck", false);
        }
        lastIterationPosition = currentArmPosition;

        double currentTime = limitTimer.get();
        int timeLimitLeft = (int) (MAX_HOLD_ARM_TIME_SEC - currentTime);
        SmartDashboard.putNumber("ArmCommandTimeToLimit", timeLimitLeft);

        if (isInTarget && position <= RobotMap.ARM_ANGLE_BEFORE_STOP) {
            DriverStation.reportWarning("Arm reached position at or below hold threshold, dropping", false);
            stopHolding();

            return; // make sure not to reach the following ifs
        }

        if (timeLimitLeft <= 0) {
            // todo: beware of limittimer looper. If the timer elapses while trying to set down the arm then
            //  we'll get a loop of the timer keep trying to set down the arm
            DriverStation.reportWarning("Arm reached maximum hold time, setting down", false);
            gentlyDrop();
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
