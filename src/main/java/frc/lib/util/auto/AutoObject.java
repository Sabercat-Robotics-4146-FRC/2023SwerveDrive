package frc.lib.util.auto;

import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.annotations.Expose;
import com.google.gson.reflect.TypeToken;

public class AutoObject {
    @Expose
    public Map<String, Object> command;

    public AutoObject(String path) {
        this(Path.of(path));
    }

    public AutoObject(Path path) {
        String json = "";
        try {
            json = Files.readString(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            e.printStackTrace();
        }

        Gson gson = new GsonBuilder()
                        .excludeFieldsWithoutExposeAnnotation()
                        .create();

        Type type = new TypeToken<AutoObject>() {}.getType();
        AutoObject auto = gson.fromJson(json, type);

        this.command = auto.command;
    }

    public CommandObject getCommand() {
        return new CommandObject((String) command.get("type"), (Map<String, Object>) command.get("data"));
    }
}
