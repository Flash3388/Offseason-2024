package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.UpAndDown;
import frc.robot.subsystems.Climb;
import frc.robot.commands.DriveWithXBox;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
    private Climb climb;
    private Swerve swerve;
    private XboxController xboxController;

    @Override
    public void robotInit() {
        xboxController =new XboxController(0);
        climb= new Climb();
        new JoystickButton(xboxController, XboxController.Button.kA.value)
                .onTrue(new UpAndDown(climb, true));
        swerve = SystemFactory.createSwerve();
                DriveWithXBox driveWithXBox = new DriveWithXBox(swerve,xboxController);
        swerve.setDefaultCommand(driveWithXBox);

    new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(new UpAndDown(climb,false));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void autonomousInit() {
        ReplanningConfig replanningConfig = new ReplanningConfig(
                false,
                false);
        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(0.5,0,0.00007),
                new PIDConstants(0.5,0,0.00007),
                4.4169,
                RobotMap.CHASSIS_RADIUS,
                replanningConfig
        );
        PathPlannerPath pathS = PathPlannerPath.fromPathFile("Off-season-check");
        FollowPathHolonomic pathHolonomic = new FollowPathHolonomic(
                pathS,
                swerve::getPose,
                swerve::getSpeeds,
                swerve::drive,
                holonomicPathFollowerConfig,
                ()-> {return false;},
                swerve);
        pathHolonomic.schedule();

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        climb.print();
    }

    @Override
    public void simulationPeriodic() {

    }
}
