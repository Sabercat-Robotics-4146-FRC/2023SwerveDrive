package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.defaults.*;
import frc.robot.subsystems.*;
import frc.robot.utils.Axis;
import frc.robot.utils.CommandUtil;


public class RobotContainer {
    public final CommandXboxController primaryController = new CommandXboxController(0);
    public final CommandXboxController secondaryController = new CommandXboxController(1);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

    private static RobotContainer instance;
    public static synchronized RobotContainer getInstance() {
    if (instance == null) {
        instance = new RobotContainer();
    }
        return instance;
    }

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);

        drivetrainSubsystem.setDefaultCommand(
            new DriveCommand(
                drivetrainSubsystem, 
                new Axis(() -> primaryController.getLeftY(), 3.25),
                new Axis(() -> primaryController.getLeftX(), 3.25),
                new Axis(() -> primaryController.getRightX(), 1.5)
            )
        );
 
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.a().onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
        primaryController.start().onTrue(Commands.runOnce(drivetrainSubsystem.gyroscope::reset));
        primaryController.back().onTrue(Commands.runOnce(drivetrainSubsystem::resetGyro180));
    
    }
 
    public Command getAutonomousCommand() {
        return CommandUtil.getInstance().getCommand(this, "frc.robot.autos.AutoCommand");
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return this.drivetrainSubsystem;
    }

}