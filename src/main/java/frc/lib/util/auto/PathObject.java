package frc.lib.util.auto;

import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.annotations.Expose;
import com.google.gson.reflect.TypeToken;

public class PathObject {
    @Expose
    public List<Waypoint> waypoints;
    @Expose
    public List<EventMarker> eventMarkers;

    public String pathName;

    public PathObject(Path path, String pathName) {
        this(path);
        this.pathName = pathName;
    }
 
    public PathObject(Path path) {
        String json = "";
        try {
            json = Files.readString(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            e.printStackTrace();
        }

        Gson gson = new GsonBuilder()
                        .excludeFieldsWithoutExposeAnnotation()
                        .create();

        Type type = new TypeToken<PathObject>() {}.getType();
        PathObject pathObject = gson.fromJson(json, type);

        waypoints = pathObject.waypoints;
        eventMarkers = pathObject.eventMarkers;
    }

    public class Waypoint {
        @Expose
        public Map<String, Double> anchor;
        @Expose
        public Map<String, Double> prevControl;
        @Expose
        public Map<String, Double> nextControl;
    }

    public class EventMarker {
        @Expose 
        public String name;
        @Expose
        public double waypointRelativePos;
        @Expose
        public CommandObject command;
    }
}

