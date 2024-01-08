package frc.lib.util.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.autos.PathCommand;
import frc.robot.utils.CommandUtil;

import com.google.gson.annotations.Expose;

public class CommandObject {
    @Expose
    public String type;
    @Expose
    public Map<String, Object> data;

    public List<CommandObject> commands;

    public CommandObject(String type, Map<String, Object> data) {
        this.type = type;
        this.data = data;

        if(isCommandGroup()) {
            evalData();
        }
    }

    public Command eval() {
        Command command = new InstantCommand();

        if(!isCommandGroup()) {

            if(type.equals("named")) {
                command = CommandUtil.getInstance().getCommand(null, ((String) data.get("path")).replaceAll("/","."));
            } else if(type.equals("wait")) {
                command = new WaitCommand((double) data.get("waitTime"));
            } else if(type.equals("path")) {
                String pathName = (String) data.get("pathName");
                command = new PathCommand(
                    AutoConstants.PATH_DIR.toString() + "/" + pathName + ".path");
            }
        }
        else {
            if(commands == null) evalData();
            List<Command> commandList = new ArrayList<>();

            for(CommandObject com : commands) commandList.add(com.eval());

            switch(type) {
                case "sequential":
                    command = new SequentialCommandGroup(commandList.toArray(new Command[commandList.size()]));
                    break;
                case "parallel":
                    command = new ParallelCommandGroup(commandList.toArray(new Command[commandList.size()]));
                    break;
                case "deadline": 
                    command = new ParallelDeadlineGroup(commandList.remove(0), commandList.toArray(new Command[commandList.size()-1]));
                    break;
                case "race":
                    command = new ParallelRaceGroup(commandList.toArray(new Command[commandList.size()]));
                    break;
            }
            
        }

        return command;
    }

    private void evalData() {
        commands = new ArrayList<>();
        List<Map<String, Object>> l = (List<Map<String, Object>>) data.get("commands");
        for(Map<String, Object> o : (List<Map<String, Object>>) l) {
            CommandObject c = new CommandObject((String) o.get("type"), (Map<String, Object>) o.get("data"));
            commands.add(c);
        }
    }

    private boolean isCommandGroup() {
        return type.equals("sequential") || 
                type.equals("parallel") || 
                type.equals("deadline") || 
                type.equals("race");
    }
}
