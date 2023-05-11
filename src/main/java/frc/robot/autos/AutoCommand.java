package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoCommand extends SequentialCommandGroup{
    public AutoCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {

        PIDController xController = new PIDController(DriveConstants.drivePIDF[0], DriveConstants.drivePIDF[1], DriveConstants.drivePIDF[2]);
        PIDController yController = new PIDController(DriveConstants.drivePIDF[0], DriveConstants.drivePIDF[1], DriveConstants.drivePIDF[2]);
        ProfiledPIDController rController = new ProfiledPIDController(
            DriveConstants.anglePIDF[0], DriveConstants.anglePIDF[1], DriveConstants.anglePIDF[2], 
            AutoConstants.rotationalConstraints);

        rController.enableContinuousInput(-180, 180);

        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
            new InstantCommand(() -> drivetrain.getGyro().reset()),
            new SwerveControllerCommand(
                trajectory,
                drivetrain::getPose,
                DriveConstants.swerveKinematics,
                xController,
                yController,
                rController,
                drivetrain::updateModules, 
                drivetrain
            )
        );
    }
    
}
