package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;

public class CommandUtil {
    public Map<Class<?>, Class<?>> wrapper = new HashMap<>();

    private static CommandUtil instance;
    public static synchronized CommandUtil getInstance() {
    if (instance == null) {
        instance = new CommandUtil();
    }
        return instance;
    }

    public CommandUtil() {
        wrapper.put(boolean.class, Boolean.class);
        wrapper.put(byte.class, Byte.class);
        wrapper.put(short.class, Short.class);
        wrapper.put(char.class, Character.class);
        wrapper.put(int.class, Integer.class);
        wrapper.put(long.class, Long.class);
        wrapper.put(float.class, Float.class);
        wrapper.put(double.class, Double.class);
    }
    
    public Command getCommand(RobotContainer container, String str) {
        String[] strs = str.split(",\\s");

        try {
            Class<?> c = Class.forName(strs[0]);
            // for(Constructor<?> cons : c.getConstructors()) {
            //     if(cons.getParameterTypes().length != strs.length-1) continue;

            //     boolean flag = false;
            //     Class<?>[] parameters = cons.getParameterTypes();
            //     Object[] args = new Object[parameters.length];
            //     args[0] = container;
            //     for(int i = 1; i < parameters.length; i++) {
            //         Class<?> type = cons.getParameterTypes()[i];
            //         if(type.isPrimitive()) type = wrapper.get(type);
            //         try {
            //             Object o = type.getDeclaredMethod("valueOf", String.class).invoke(null, strs[i]);
            //             args[i] = o;
            //         } catch(Exception e) {
            //             System.out.println("Parameters Not Matching");
            //             //e.printStackTrace();
            //             flag = true;
            //             break;
            //         }
            //     }
            //     if(flag) continue;
            //     return (Command) cons.newInstance(args);
            // }  
            System.out.println("THIS WORKS: " + str);
            return (Command) c.getDeclaredConstructor().newInstance();
        } catch (Exception e) {
            System.out.println("bros on something fr");
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        return new InstantCommand();
    }
}