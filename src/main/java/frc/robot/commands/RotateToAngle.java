package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;

public class RotateToAngle extends Command {

    private final Swerve swerve;
    private final double targetAngleDegrees;
    private final PIDController pidController;
    private final double KP = 0.015;
    private final double KI = 0.01;
    private final double KD = 0;
    private final double POSITION_TOLERANCE = 1;
    private final double VELOCITY_TOLERANCE = 0.5;

    public RotateToAngle(Swerve swerve, double targetAngleDegrees) {
        this.swerve = swerve;
        this.targetAngleDegrees = targetAngleDegrees;

        pidController = new PIDController(KP,KI,KD);
        pidController.setTolerance(POSITION_TOLERANCE,VELOCITY_TOLERANCE);
        pidController.enableContinuousInput(0, 360);
        pidController.setIZone(10);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Rotation2d getCurrentRotaion = swerve.getHeadingDegrees();
        double getCurrentAngle = getCurrentRotaion.getDegrees();
        double speed = pidController.calculate(getCurrentAngle, targetAngleDegrees);
        swerve.drive(0, 0, speed * RobotMap.ATTAINBLE_MAX_SPEED_MPS_SWERVE);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
