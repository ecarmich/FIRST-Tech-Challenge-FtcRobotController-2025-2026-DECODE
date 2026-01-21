package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.linear.*;

public class KalmanFilterXYThetaWithErrorStdDevsSensor {
    // State: [x, y, theta, vx, vy, omega]
    private RealVector state;
    private RealMatrix P; // Error Covariance
    private double lastTimestampMillis;

    public KalmanFilterXYThetaWithErrorStdDevsSensor(double initialX, double initialY, double initialTheta, long startTime) {
        // 1. Initialize State
        state = new ArrayRealVector(new double[] { initialX, initialY, initialTheta, 0, 0, 0 });

        // 2. Initialize Uncertainty (P)
        // High uncertainty for velocity (we don't know it yet), low for position
        P = MatrixUtils.createRealIdentityMatrix(6);
        P.setEntry(3, 3, 100);
        P.setEntry(4, 4, 100);
        P.setEntry(5, 5, 100);

        this.lastTimestampMillis = startTime;
    }

    /**
     * Call this method when you receive a new sensor packet.
     * @param timestampMillis Current time (same unit as startTime)
     * @param measX Measured X
     * @param measY Measured Y
     * @param measTheta Measured Theta (radians)
     * @param stdX Standard deviation of X error (from sensor spec)
     * @param stdY Standard deviation of Y error (from sensor spec)
     * @param stdTheta Standard deviation of Theta error (from sensor spec)
     */
    public void update(double timestampMillis, double measX, double measY, double measTheta,
                       double stdX, double stdY, double stdTheta) {

        // --- 1. PREDICT STEP ---
        double dt = (timestampMillis - lastTimestampMillis) / 1000.0; // Convert to seconds
        if (dt > 0) {
            // Create Dynamic State Transition Matrix (A) based on dt
            // x_new = x + vx * dt
            RealMatrix A = MatrixUtils.createRealIdentityMatrix(6);
            A.setEntry(0, 3, dt);
            A.setEntry(1, 4, dt);
            A.setEntry(2, 5, dt);

            // Create Process Noise Matrix (Q)
            // This represents uncertainty in the *model* (e.g., wheel slip, friction)
            // We scale it by dt because uncertainty grows with time
            RealMatrix Q = MatrixUtils.createRealIdentityMatrix(6).scalarMultiply(0.01 * dt);

            // x = A * x
            state = A.operate(state);

            // P = A * P * A^T + Q
            P = A.multiply(P).multiply(A.transpose()).add(Q);

            // Normalize predicted theta to -PI to +PI
            double theta = state.getEntry(2);
            state.setEntry(2, normalizeAngle(theta));
        }
        lastTimestampMillis = timestampMillis;

        // --- 2. UPDATE STEP ---

        // Measurement Matrix (H): We observe x, y, theta (indices 0, 1, 2)
        RealMatrix H = MatrixUtils.createRealMatrix(3, 6);
        H.setEntry(0, 0, 1);
        H.setEntry(1, 1, 1);
        H.setEntry(2, 2, 1);

        // Measurement Noise Matrix (R): From your sensor's standard deviations
        RealMatrix R = MatrixUtils.createRealDiagonalMatrix(new double[] {
                Math.pow(stdX, 2), Math.pow(stdY, 2), Math.pow(stdTheta, 2)
        });

        // Calculate Innovation (y = z - Hx)
        RealVector z = new ArrayRealVector(new double[] { measX, measY, measTheta });
        RealVector predictedMeas = H.operate(state);
        RealVector innovation = z.subtract(predictedMeas);

        // --- CUSTOM FIX: Handle Theta Wrap-Around ---
        // If the difference is e.g. 350 degrees, it's actually -10 degrees.
        double thetaErr = innovation.getEntry(2);
        thetaErr = normalizeAngle(thetaErr); // Commons Math utility
        innovation.setEntry(2, thetaErr);

        // Calculate Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
        RealMatrix Ht = H.transpose();
        RealMatrix S = H.multiply(P).multiply(Ht).add(R); // Innovation Covariance

        // Use LUDecomposition for efficient inversion
        RealMatrix K = P.multiply(Ht).multiply(new LUDecomposition(S).getSolver().getInverse());

        // Update State: x = x + K * y
        state = state.add(K.operate(innovation));

        // Update Covariance: P = (I - K * H) * P
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(6);
        P = I.subtract(K.multiply(H)).multiply(P);

        // Final normalization of estimated theta
        state.setEntry(2, normalizeAngle(state.getEntry(2)));
    }

    public double getX() {
        return state.getEntry(0);
    }

    public double getY() {
        return state.getEntry(1);
    }

    public double getTheta() {
        return state.getEntry(2);
    }

    /**
     * Predicts the robot's state at a specific future (or current) time
     * without modifying the filter's internal state.
     * * @param queryTimestampMillis The time you want the prediction for (same units as startTime)
     * @return A double array [x, y, theta] representing the predicted pose
     */
    public double[] getPredictedPose(double queryTimestampMillis) {
        double dt = (queryTimestampMillis - lastTimestampMillis) / 1000.0;

        // If the query is for the exact same time as the last update, just return current state
        if (dt <= 0) {
            return new double[] { getX(), getY(), getTheta() };
        }

        // 1. Create the State Transition Matrix (A) for this specific dt
        // x_new = x + vx * dt
        RealMatrix A = MatrixUtils.createRealIdentityMatrix(6);
        A.setEntry(0, 3, dt);
        A.setEntry(1, 4, dt);
        A.setEntry(2, 5, dt);

        // 2. Project the state forward
        // We don't need to update P (covariance) here because we aren't saving this state
        RealVector predictedState = A.operate(state);

        // 3. Normalize Theta
        double predictedTheta = normalizeAngle(predictedState.getEntry(2));

        return new double[] {
                predictedState.getEntry(0),
                predictedState.getEntry(1),
                predictedTheta
        };
    }

    private static double normalizeAngle(double theta) {
        while (theta > 180) {
            theta -= 360;
        }
        while (theta <= -180) {
            theta += 360;
        }
        return theta;
    }
}