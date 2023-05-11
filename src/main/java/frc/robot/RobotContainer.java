package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.*;
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

        //primaryController.x().toggleOnTrue(new PathAuto(drivetrainSubsystem));


        primaryController.a().onTrue(Commands.runOnce(drivetrainSubsystem::toggleFieldOriented));
        primaryController.b().toggleOnTrue(Commands.run(()-> drivetrainSubsystem.drive(new Translation2d(1, 0), 1)));
        primaryController.start().onTrue(Commands.runOnce(drivetrainSubsystem.gyroscope::reset));
        
    }

 
    public Command getAutonomousCommand() {
        return new SimpleAuto(drivetrainSubsystem);
    }
}
