package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SimpleAuto extends SequentialCommandGroup {
    public SimpleAuto(DrivetrainSubsystem drivetrain) {

        List<Translation2d> interiorPoints = new ArrayList<>();
        interiorPoints.add(new Translation2d(2, -0.5));
        interiorPoints.add(new Translation2d(4, 0.5));
        Trajectory trajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                interiorPoints,
                new Pose2d(6, 0, new Rotation2d(0)),
                AutoConstants.trajectoryConfig);
        addCommands(
            new AutoCommand(drivetrain, trajectory)
        );
    }
    
}
