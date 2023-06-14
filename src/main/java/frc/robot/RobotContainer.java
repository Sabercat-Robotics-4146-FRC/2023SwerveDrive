package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utils.Axis;


public class RobotContainer {

    public final CommandXboxController primaryController = new CommandXboxController(0);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

    public RobotContainer() {
        drivetrainSubsystem.setDefaultCommand(
            new DriveCommand(
                drivetrainSubsystem, 
                new Axis(() -> primaryController.getLeftY(), 1.25),
                new Axis(() -> primaryController.getLeftX(), 1.25),
                new Axis(() -> primaryController.getRightX(), 1.0)
            )
        );
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.a().onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
        primaryController.start().onTrue(Commands.runOnce(drivetrainSubsystem.gyroscope::reset));
        
    }

 
    public Command getAutonomousCommand() {
        return null;
    }
}
