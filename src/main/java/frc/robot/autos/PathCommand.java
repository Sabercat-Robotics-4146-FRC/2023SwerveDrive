package frc.robot.autos;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.HashMap;
import java.util.LinkedList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.auto.PathObject;
import frc.lib.util.auto.TrajectoryObject;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

public class PathCommand extends CommandBase {
    private RobotContainer container;
    private PathObject path;
    private Map<Double, Command> commandMap;
    private List<Command> currentCommands;
    private FollowTrajectoryCommand trajectoryCommand;
    private Queue<Double> waypointPositions;

    private double lastWaypointPos;

    public PathCommand(String path) {
        this(Path.of(path));
    }

    public PathCommand(Path path) {
        this(new PathObject(path, path.getFileName().toString().replace(".path", "")));
    }

    public PathCommand(PathObject path) {
        this.container = RobotContainer.getInstance();
        this.path = path;

        commandMap = new HashMap<>();

        for(PathObject.EventMarker marker : path.eventMarkers) {
            commandMap.put(marker.waypointRelativePos, marker.command.eval());
        }
    }

    @Override
    public void initialize() {
        TrajectoryObject trajectory = new TrajectoryObject(AutoConstants.TRAJECTORY_DIR.resolve(path.pathName + ".json"));

        System.out.println(AutoConstants.TRAJECTORY_DIR.toString() + ("/" + path.pathName + ".json"));
        System.out.println(AutoConstants.TRAJECTORY_DIR.resolve(path.pathName + ".json"));

        trajectoryCommand = new FollowTrajectoryCommand(container, trajectory);
        currentCommands = new ArrayList<>();

        /*
         * get the position of the waypoint along the path by 
         * iterating through the trajectory states
         */
        waypointPositions = new LinkedList<>();
        int i = 0;
        for(TrajectoryObject.State state : trajectory.trajectory) {
            if(path.waypoints.get(i).anchor.get("x") == state.pose.getX() && 
               path.waypoints.get(i).anchor.get("y") == state.pose.getY()) {

                waypointPositions.add(state.test);
                i++;
            }
        }

        lastWaypointPos = 0;
    }

    @Override
    public void execute() {
        /*
         * schedule trajectory command if it has not been scheduled yet
         */
        if(!trajectoryCommand.isScheduled() && !trajectoryCommand.isFinished()) {
            trajectoryCommand.schedule();
        } 

        /*
         * if the current waypoint is passed, update the queue and the 
         * previous value
         */
        if(trajectoryCommand.getPosition() > waypointPositions.peek()) {
            lastWaypointPos = waypointPositions.poll();
        }

        /*
         * get the relative waypoint position of the current point on
         * the trajectory in a ratio of the current distance travelled
         * over the total distance to travel
         */
        double relativePosition = path.waypoints.size() - waypointPositions.size() -
            (waypointPositions.peek() - trajectoryCommand.getPosition())/
            (waypointPositions.peek() - lastWaypointPos);

        /*
         * get the next command by checking the relative position mapping
         * and rounding the current position to the closest 0.05
         */
        Command nextCommand = commandMap.get(Math.round(relativePosition * 20) / 20.0);

        if(nextCommand != null) System.out.println(nextCommand.getClass());

        /*
         * loop through all current commands and evaluate
         * each command
         */
        for(Command command : currentCommands) {
            // if the command is finished, remove it from current commands
            if(command.isFinished()) {
                currentCommands.remove(command);
                continue;
            }

            // check if next command has overlapping subsystems
            // if it does, end the commands that overlap
            if(nextCommand != null && !currentCommands.contains(nextCommand) && !Collections.disjoint(command.getRequirements(), nextCommand.getRequirements())) {
                command.end(false);
                currentCommands.remove(command);
            }   
        }

        /*
         * If the current command is not null, meaning it exists,
         * schedule it and add it to the currentCommands list
         */
        if(nextCommand != null && !currentCommands.contains(nextCommand)) {
            currentCommands.add(nextCommand);
            nextCommand.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        for(Command command : currentCommands) {
            command.end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return trajectoryCommand.isFinished();
    }
}
