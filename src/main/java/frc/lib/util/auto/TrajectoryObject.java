package frc.lib.util.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import java.lang.reflect.Type;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.annotations.Expose;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryObject {
    @Expose
    public List<TrajectoryObject.State> trajectory;

    public TrajectoryObject() {
        trajectory = new ArrayList<>();
    }

    public TrajectoryObject(List<TrajectoryObject.State> list) {
        this.trajectory = list;
    }

    public TrajectoryObject(String path){
        this(Path.of(path));
    }

    public TrajectoryObject(Path path) {
        String json = "";
        try {
            json = Files.readString(path, StandardCharsets.UTF_8);
        } catch (IOException e) {

        }

        Gson gson = new GsonBuilder().excludeFieldsWithoutExposeAnnotation().create();

        Type type = new TypeToken<List<TrajectoryObject.State>>() {}.getType();
        trajectory = gson.fromJson(json, type);
    }

    private static double lerp(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    private static Pose2d lerp(Pose2d startValue, Pose2d endValue, double t) {
        return startValue.plus((endValue.minus(startValue)).times(t));
    }

    public State sample(double time) {
        if (time <= trajectory.get(0).time) {
          return trajectory.get(0);
        }
        if (time >= trajectory.get(trajectory.size()-1).time) {
          return trajectory.get(trajectory.size() - 1);
        }
    
        // To get the element that we want, we will use a binary search algorithm
        // instead of iterating over a for-loop. A binary search is O(std::log(n))
        // whereas searching using a loop is O(n).
    
        // This starts at 1 because we use the previous state later on for
        // interpolation.
        int low = 1;
        int high = trajectory.size() - 1;
    
        while (low != high) {
          int mid = (low + high) / 2;
          if (trajectory.get(mid).time < time) {
            // This index and everything under it are less than the requested
            // timestamp. Therefore, we can discard them.
            low = mid + 1;
          } else {
            // t is at least as large as the element at this index. This means that
            // anything after it cannot be what we are looking for.
            high = mid;
          }
        }
    
        // High and Low should be the same.
    
        // The sample's timestamp is now greater than or equal to the requested
        // timestamp. If it is greater, we need to interpolate between the
        // previous state and the current state to get the exact state that we
        // want.
        final State sample = trajectory.get(low);
        final State prevSample = trajectory.get(low - 1);
    
        // If the difference in states is negligible, then we are spot on!
        if (Math.abs(sample.time - prevSample.time) < 1E-9) {
          return sample;
        }
        // Interpolate between the two states for the state that we want.
        return prevSample.interpolate(
            sample,
            (time - prevSample.time) / (sample.time - prevSample.time));
    }

    public class State {
        @Expose
        public double time;
        @Expose
        public Pose pose;
        @Expose
        public double velocity;
        @Expose
        public double holonomicRotation;
        @Expose
        public double test;
        
        public double acceleration;
        public double holonomicAngularVelocity;

        public State(double time, Pose pose, double velocity, double acceleration, double holonomicRotation, double holonomicAngularVelocity, double test) {
            this.time = time;
            this.pose = pose;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.holonomicRotation = holonomicRotation;
            this.holonomicAngularVelocity = holonomicAngularVelocity;
            this.test = test;
        }

        State interpolate(State endValue, double i) {
            // Find the new t value.
            final double newT = lerp(time, endValue.time, i);
      
            // Find the delta time between the current state and the interpolated state.
            final double deltaT = newT - time;
      
            // If delta time is negative, flip the order of interpolation.
            if (deltaT < 0) {
              return endValue.interpolate(this, 1 - i);
            }

            acceleration = (endValue.velocity - velocity) / deltaT;

            // calculate the holonomic angular acceleration
            double holonomicAngularAcceleration = (endValue.holonomicAngularVelocity - holonomicAngularVelocity)/(endValue.time - time);
      
            // Check whether the robot is reversing at this stage.
            final boolean reversing =
                velocity < 0
                    || Math.abs(velocity) < 1E-9 && acceleration < 0;

            // if holonomic angle is reversing
            final boolean reversingAngle =
                holonomicAngularVelocity < 0
                    || Math.abs(holonomicAngularVelocity) < 1E-9 && holonomicAngularAcceleration < 0;
      
            // Calculate the new velocity
            // v_f = v_0 + at
            final double newV = velocity + (acceleration * deltaT);

            // new angular velocity
            // av_f = av_0 + (aa)(t)

            // new holonomic anglular velocity
            final double newHAV = (endValue.holonomicRotation - holonomicRotation) / deltaT + holonomicAngularAcceleration *deltaT;
      
            // Calculate the change in position.
            // delta_s = v_0 t + 0.5at²
            final double newS =
                (velocity * deltaT
                        + 0.5 * acceleration * Math.pow(deltaT, 2))
                    * (reversing ? -1.0 : 1.0);

            // calculate the change in holonomic angle position
            final double newSA = 
                (holonomicAngularVelocity * deltaT
                        + 0.5 * holonomicAngularAcceleration * Math.pow(deltaT, 2))
                    * (reversingAngle ? -1.0 : 1.0);
      
            // Return the new state. To find the new position for the new state, we need
            // to interpolate between the two endpoint poses. The fraction for
            // interpolation is the change in position (delta s) divided by the total
            // distance between the two endpoints.
            final double interpolationFrac =
                newS / endValue.getPose().getTranslation().getDistance(pose.getPose().getTranslation());

            final double interpolationFracAngle = 
                newSA / Math.abs(endValue.holonomicRotation - holonomicRotation);
      
            return new State(
                newT,
                toPose(lerp(pose.getPose(), endValue.pose.getPose(), interpolationFrac)),
                newV,
                acceleration,
                lerp(holonomicRotation, endValue.holonomicRotation, interpolationFracAngle),
                newHAV,
                lerp(test, endValue.test, i));

          }

          private Pose toPose(Pose2d pose) {
            return new Pose(pose);
          }

          public Pose2d getPose() {
            return pose.getPose();
          }
    }

    public class Pose {
        @Expose
        public Map<String, Double> rotation;
        @Expose
        public Map<String, Double> translation;

        public Pose(Pose2d pose) {
            this.rotation = Map.of("radians", pose.getRotation().getRadians());
            this.translation = Map.of("x", pose.getX(), "y", pose.getY());
        }

        public Pose2d getPose() {
            return new Pose2d(new Translation2d(translation.get("x"), translation.get("y")), new Rotation2d(rotation.get("radians")));
        }

        public double getX() {
            return translation.get("x");
        }

        public double getY() {
            return translation.get("y");
        }

        public double getRotation() {
            return rotation.get("radians");
        }
 
        public double distanceTo(Pose pose) {
            return Math.sqrt(Math.pow(pose.getX() - getX(), 2) + Math.pow(pose.getY() - getY(), 2));
        }
    }
}