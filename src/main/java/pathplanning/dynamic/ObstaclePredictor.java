package pathplanning.dynamic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.LinkedList;

/**
 * Predicts obstacle motion using various filtering techniques
 */
public class ObstaclePredictor {
    
    /**
     * Simple Kalman filter for obstacle prediction
     */
    public static class KalmanFilter {
        private double[] state; // [x, y, vx, vy]
        private double[][] covariance;
        private final double processNoise;
        private final double measurementNoise;
        
        public KalmanFilter(Pose2d initialPose) {
            this(initialPose, 0.1, 0.5);
        }
        
        public KalmanFilter(Pose2d initialPose, double processNoise, 
                           double measurementNoise) {
            this.state = new double[]{
                initialPose.getX(), initialPose.getY(), 0, 0
            };
            this.covariance = new double[4][4];
            for (int i = 0; i < 4; i++) {
                covariance[i][i] = 1.0;
            }
            this.processNoise = processNoise;
            this.measurementNoise = measurementNoise;
        }
        
        /**
         * Update filter with new measurement
         */
        public void update(Pose2d measurement, double dt) {
            // Prediction step
            state[0] += state[2] * dt;
            state[1] += state[3] * dt;
            
            // Add process noise to covariance
            for (int i = 0; i < 4; i++) {
                covariance[i][i] += processNoise;
            }
            
            // Update step
            double[] innovation = {
                measurement.getX() - state[0],
                measurement.getY() - state[1]
            };
            
            // Kalman gain calculation (simplified)
            double gainX = covariance[0][0] / (covariance[0][0] + measurementNoise);
            double gainY = covariance[1][1] / (covariance[1][1] + measurementNoise);
            
            // Update state
            state[0] += gainX * innovation[0];
            state[1] += gainY * innovation[1];
            
            // Update velocity estimate
            if (dt > 0.01) {
                state[2] = innovation[0] / dt;
                state[3] = innovation[1] / dt;
            }
            
            // Update covariance
            covariance[0][0] *= (1 - gainX);
            covariance[1][1] *= (1 - gainY);
        }
        
        /**
         * Predict future position
         */
        public Pose2d predict(double timeAhead) {
            double x = state[0] + state[2] * timeAhead;
            double y = state[1] + state[3] * timeAhead;
            return new Pose2d(x, y, new Rotation2d());
        }
        
        public Translation2d getVelocity() {
            return new Translation2d(state[2], state[3]);
        }
    }
    
    /**
     * Moving average filter for smoothing
     */
    public static class MovingAverageFilter {
        private final LinkedList<Pose2d> history;
        private final int windowSize;
        
        public MovingAverageFilter(int windowSize) {
            this.history = new LinkedList<>();
            this.windowSize = windowSize;
        }
        
        public void addMeasurement(Pose2d pose) {
            history.add(pose);
            if (history.size() > windowSize) {
                history.removeFirst();
            }
        }
        
        public Pose2d getSmoothed() {
            if (history.isEmpty()) {
                return null;
            }
            
            double sumX = 0;
            double sumY = 0;
            for (Pose2d pose : history) {
                sumX += pose.getX();
                sumY += pose.getY();
            }
            
            return new Pose2d(
                sumX / history.size(),
                sumY / history.size(),
                history.getLast().getRotation()
            );
        }
    }
}