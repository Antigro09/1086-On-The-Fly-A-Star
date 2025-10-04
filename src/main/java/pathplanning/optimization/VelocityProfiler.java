package pathplanning.optimization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import pathplanning.util.Path;

import java.util.ArrayList;
import java.util.List;

/**
 * Generates time-parameterized velocity profiles for paths
 */
public class VelocityProfiler {
    private final double maxVelocity;
    private final double maxAcceleration;
    private final double maxAngularVelocity;
    private final double dt;
    
    public VelocityProfiler(double maxVelocity, double maxAcceleration,
                           double maxAngularVelocity) {
        this(maxVelocity, maxAcceleration, maxAngularVelocity, 0.02);
    }
    
    public VelocityProfiler(double maxVelocity, double maxAcceleration,
                           double maxAngularVelocity, double dt) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.dt = dt;
    }
    
    /**
     * Generate trajectory with velocity profile
     */
    public Trajectory generateProfile(Path path, Pose2d currentPose) {
        if (path.size() < 2) {
            return null;
        }
        
        List<TrajectoryState> states = new ArrayList<>();
        
        // Calculate distances between waypoints
        List<Double> distances = new ArrayList<>();
        double totalDistance = 0;
        
        for (int i = 0; i < path.size() - 1; i++) {
            double dist = path.get(i).getTranslation()
                .getDistance(path.get(i + 1).getTranslation());
            distances.add(dist);
            totalDistance += dist;
        }
        
        // Generate velocity profile using trapezoidal profile
        double currentVelocity = 0;
        double currentTime = 0;
        double distanceTraveled = 0;
        
        // Add initial state
        states.add(new TrajectoryState(currentTime, path.get(0), 
                                      currentVelocity, 0, 0));
        
        int waypointIndex = 0;
        
        while (waypointIndex < path.size() - 1) {
            double segmentDistance = distances.get(waypointIndex);
            double distanceInSegment = 0;
            
            while (distanceInSegment < segmentDistance && 
                   waypointIndex < path.size() - 1) {
                
                // Calculate target velocity based on remaining distance
                double remainingDistance = totalDistance - distanceTraveled;
                double brakingDistance = (currentVelocity * currentVelocity) / 
                                        (2 * maxAcceleration);
                
                double targetVelocity = maxVelocity;
                
                if (brakingDistance >= remainingDistance) {
                    // Need to start braking
                    targetVelocity = Math.sqrt(2 * maxAcceleration * remainingDistance);
                }
                
                // Apply acceleration limits
                double velocityChange = Math.min(
                    maxAcceleration * dt,
                    Math.abs(targetVelocity - currentVelocity)
                );
                
                if (targetVelocity > currentVelocity) {
                    currentVelocity += velocityChange;
                } else {
                    currentVelocity -= velocityChange;
                }
                
                currentVelocity = Math.max(0, Math.min(maxVelocity, currentVelocity));
                
                // Update position
                double stepDistance = currentVelocity * dt;
                distanceInSegment += stepDistance;
                distanceTraveled += stepDistance;
                currentTime += dt;
                
                // Interpolate pose along segment
                double t = Math.min(1.0, distanceInSegment / segmentDistance);
                Pose2d pose = path.get(waypointIndex).interpolate(
                    path.get(waypointIndex + 1), t
                );
                
                // Calculate acceleration
                double acceleration = velocityChange / dt;
                
                states.add(new TrajectoryState(currentTime, pose, 
                                              currentVelocity, acceleration, 0));
                
                if (distanceInSegment >= segmentDistance) {
                    waypointIndex++;
                    distanceInSegment = 0;
                }
            }
        }
        
        // Ensure final state has zero velocity
        if (states.get(states.size() - 1).getVelocity() > 0.01) {
            states.add(new TrajectoryState(
                currentTime + dt,
                path.get(path.size() - 1),
                0, 0, 0
            ));
        }
        
        return new Trajectory(states);
    }
    
    /**
     * Trajectory state at a point in time
     */
    public static class TrajectoryState {
        private final double time;
        private final Pose2d pose;
        private final double velocity;
        private final double acceleration;
        private final double angularVelocity;
        
        public TrajectoryState(double time, Pose2d pose, double velocity,
                              double acceleration, double angularVelocity) {
            this.time = time;
            this.pose = pose;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.angularVelocity = angularVelocity;
        }
        
        public double getTime() { return time; }
        public Pose2d getPose() { return pose; }
        public double getVelocity() { return velocity; }
        public double getAcceleration() { return acceleration; }
        public double getAngularVelocity() { return angularVelocity; }
        
        public double[] toArray() {
            return new double[]{
                time,
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians(),
                velocity,
                acceleration,
                angularVelocity
            };
        }
    }
    
    /**
     * Complete trajectory
     */
    public static class Trajectory {
        private final List<TrajectoryState> states;
        
        public Trajectory(List<TrajectoryState> states) {
            this.states = states;
        }
        
        public List<TrajectoryState> getStates() {
            return states;
        }
        
        public double getTotalTime() {
            return states.isEmpty() ? 0 : states.get(states.size() - 1).getTime();
        }
        
        /**
         * Sample trajectory at given time
         */
        public TrajectoryState sample(double time) {
            if (states.isEmpty()) {
                return null;
            }
            
            if (time <= 0) {
                return states.get(0);
            }
            
            if (time >= getTotalTime()) {
                return states.get(states.size() - 1);
            }
            
            // Find states to interpolate between
            for (int i = 0; i < states.size() - 1; i++) {
                TrajectoryState current = states.get(i);
                TrajectoryState next = states.get(i + 1);
                
                if (time >= current.getTime() && time <= next.getTime()) {
                    double t = (time - current.getTime()) / 
                              (next.getTime() - current.getTime());
                    
                    return interpolate(current, next, t);
                }
            }
            
            return states.get(states.size() - 1);
        }
        
        private TrajectoryState interpolate(TrajectoryState s1, TrajectoryState s2, double t) {
            Pose2d pose = s1.getPose().interpolate(s2.getPose(), t);
            double velocity = s1.getVelocity() + t * (s2.getVelocity() - s1.getVelocity());
            double acceleration = s1.getAcceleration() + 
                                t * (s2.getAcceleration() - s1.getAcceleration());
            double angularVelocity = s1.getAngularVelocity() + 
                                    t * (s2.getAngularVelocity() - s1.getAngularVelocity());
            double time = s1.getTime() + t * (s2.getTime() - s1.getTime());
            
            return new TrajectoryState(time, pose, velocity, acceleration, angularVelocity);
        }
        
        /**
         * Convert to flat array for network transmission
         */
        public double[] toDoubleArray() {
            List<Double> flat = new ArrayList<>();
            
            // Add number of states
            flat.add((double) states.size());
            
            // Add each state
            for (TrajectoryState state : states) {
                for (double val : state.toArray()) {
                    flat.add(val);
                }
            }
            
            return flat.stream().mapToDouble(Double::doubleValue).toArray();
        }
        
        /**
         * Create trajectory from flat array
         */
        public static Trajectory fromDoubleArray(double[] data) {
            if (data.length < 1) {
                return null;
            }
            
            int numStates = (int) data[0];
            List<TrajectoryState> states = new ArrayList<>();
            
            int idx = 1;
            for (int i = 0; i < numStates; i++) {
                if (idx + 6 >= data.length) {
                    break;
                }
                
                double time = data[idx++];
                double x = data[idx++];
                double y = data[idx++];
                double rotation = data[idx++];
                double velocity = data[idx++];
                double acceleration = data[idx++];
                double angularVelocity = data[idx++];
                
                Pose2d pose = new Pose2d(x, y, new Rotation2d(rotation));
                states.add(new TrajectoryState(time, pose, velocity, 
                                              acceleration, angularVelocity));
            }
            
            return new Trajectory(states);
        }
    }
}