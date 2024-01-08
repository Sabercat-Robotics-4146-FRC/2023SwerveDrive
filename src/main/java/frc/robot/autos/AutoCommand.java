package frc.robot.autos;

import frc.lib.util.auto.AutoObject;
import frc.lib.util.auto.CommandObject;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;

public class AutoCommand extends SequentialCommandGroup {    
    public AutoCommand() {
        AutoObject auto = new AutoObject(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/autos/team/New Auto Blue.auto"));

        CommandObject c = auto.getCommand();

        // add the evaluated auto command
        addCommands(c.eval());
    }
}
