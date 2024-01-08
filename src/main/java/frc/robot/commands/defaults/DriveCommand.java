package frc.robot.commands.defaults;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.Axis;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveCommand extends CommandBase {    
    private DrivetrainSubsystem drivetrain;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;


    public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation) {

        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.driveRelative(
            new Translation2d(-forward.get(true), -strafe.get(true)), 
            -rotation.get(true));
            
    }
}