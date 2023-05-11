package frc.robot.autos;

import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class PathAuto extends SequentialCommandGroup {
    public PathAuto(DrivetrainSubsystem drivetrain) {
        Trajectory trajectory = new Trajectory();
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/Test2.wpilib.json"));
        } catch (IOException e) {
            e.printStackTrace();
        }

        addCommands(new AutoCommand(drivetrain, trajectory));
        
    
    }
}
