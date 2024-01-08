package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.auto.TrajectoryObject;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
    Translation2d lastPosition;
    TrajectoryObject.State lastState;
    TrajectoryObject trajectory;

    DrivetrainSubsystem drivetrain;

    Timer timer;
    
    public FollowTrajectoryCommand(RobotContainer container, TrajectoryObject trajectory) {
        this.trajectory = trajectory;
        this.drivetrain = container.getDrivetrainSubsystem();

        lastPosition = new Translation2d(0, 0);

        timer = new Timer();

        addRequirements(container.getDrivetrainSubsystem());
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        var desiredState = trajectory.sample(timer.get()); // the next state, or, the state we are attempting to acheive 
        if(lastState == null) lastState = desiredState;

        Translation2d curPosition = new Translation2d(
            desiredState.getPose().getX(),
            desiredState.getPose().getY());

        Translation2d desiredTranslation = new Translation2d(
            (curPosition.getX() - lastPosition.getX()) / (desiredState.time - lastState.time),
            (curPosition.getY() - lastPosition.getY()) / (desiredState.time - lastState.time));

        System.out.println("TIME: " + desiredState.getPose().getX());

        lastPosition = curPosition;
        lastState = desiredState;

        SmartDashboard.putNumber("X", desiredState.getPose().getX());
        drivetrain.drive(desiredTranslation, desiredState.holonomicAngularVelocity * Math.PI / 180, true);
    } 

    @Override
    public void end(boolean interrupted) {
        drivetrain.zeroDrive();
    }

    /*
     * returns the current position of the robot on the trajectory
     */
    public double getPosition() {
        return trajectory.sample(timer.get()).test;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.trajectory.get(trajectory.trajectory.size()-1).time);
    }
}
