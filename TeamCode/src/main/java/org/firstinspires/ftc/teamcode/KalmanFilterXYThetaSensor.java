package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class KalmanFilterXYThetaSensor {
    // Tuning parameter for Process Noise (Q)
    private static final double PROCESS_NOISE = 0.01;
    // These matrices are now constant, so we create them once to save CPU
    private final RealMatrix H;
    private final RealMatrix R;
    // State: [x, y, theta, vx, vy, omega]
    private RealVector state;
    private RealMatrix P; // Error Covariance
    private double lastTimestampMillis;

    /**
     * @param initialX      Initial X position
     * @param initialY      Initial Y position
     * @param initialTheta  Initial Theta
     * @param startTime     Starting timestamp
     * @param fixedStdX     The expected standard deviation of X (e.g., 1.0 inch)
     * @param fixedStdY     The expected standard deviation of Y
     * @param fixedStdTheta The expected standard deviation of Theta (radians)
     */
    public KalmanFilterXYThetaSensor(double initialX, double initialY, double initialTheta, long startTime, double fixedStdX, double fixedStdY, double fixedStdTheta) {

        // 1. Initialize State
        state = new ArrayRealVector(new double[]{initialX, initialY, initialTheta, 0, 0, 0});

        // 2. Initialize Uncertainty (P)
        P = MatrixUtils.createRealIdentityMatrix(6);
        P.setEntry(3, 3, 100); // High uncertainty for initial velocity
        P.setEntry(4, 4, 100);
        P.setEntry(5, 5, 100);

        this.lastTimestampMillis = startTime;

        // 3. Setup Constant Measurement Matrix (H)
        // We observe x, y, theta (indices 0, 1, 2)
        H = MatrixUtils.createRealMatrix(3, 6);
        H.setEntry(0, 0, 1);
        H.setEntry(1, 1, 1);
        H.setEntry(2, 2, 1);

        // 4. Setup Constant Noise Matrix (R)
        // Variances are constant now
        R = MatrixUtils.createRealDiagonalMatrix(new double[]{Math.pow(fixedStdX, 2), Math.pow(fixedStdY, 2), Math.pow(fixedStdTheta, 2)});
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

    public void update(double timestampMillis, double measX, double measY, double measTheta) {
        double dt = (timestampMillis - lastTimestampMillis) / 1000.0; // Seconds
        lastTimestampMillis = timestampMillis;

        // --- 1. PREDICT STEP ---
        if (dt > 0) {
            // A: State Transition Matrix
            RealMatrix A = MatrixUtils.createRealIdentityMatrix(6);
            A.setEntry(0, 3, dt);
            A.setEntry(1, 4, dt);
            A.setEntry(2, 5, dt);

            // Q: Process Noise (scales with dt)
            RealMatrix Q = MatrixUtils.createRealIdentityMatrix(6).scalarMultiply(PROCESS_NOISE * dt);

            // x = A * x
            state = A.operate(state);

            // P = A * P * A^T + Q
            P = A.multiply(P).multiply(A.transpose()).add(Q);

            // Normalize predicted theta
            state.setEntry(2, normalizeAngle(state.getEntry(2)));
        }

        // --- 2. UPDATE STEP ---

        // Calculate Innovation (y = z - Hx)
        RealVector z = new ArrayRealVector(new double[]{measX, measY, measTheta});
        RealVector predictedMeas = H.operate(state);
        RealVector innovation = z.subtract(predictedMeas);

        // Handle Theta Wrap-Around (Critical!)
        double thetaErr = innovation.getEntry(2);
        thetaErr = normalizeAngle(thetaErr);
        innovation.setEntry(2, thetaErr);

        // Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
        // We use the pre-allocated H and R here
        RealMatrix Ht = H.transpose();
        RealMatrix S = H.multiply(P).multiply(Ht).add(R);

        RealMatrix K = P.multiply(Ht).multiply(new LUDecomposition(S).getSolver().getInverse());

        // Update State: x = x + K * y
        state = state.add(K.operate(innovation));

        // Update Covariance: P = (I - K * H) * P
        RealMatrix I = MatrixUtils.createRealIdentityMatrix(6);
        P = I.subtract(K.multiply(H)).multiply(P);

        // Final normalization
        state.setEntry(2, normalizeAngle(state.getEntry(2)));
    }

    // Prediction method for smooth framing between updates
    public double[] getPredictedPose(double queryTimestampMillis) {
        double dt = (queryTimestampMillis - lastTimestampMillis) / 1000.0;
        if (dt <= 0) {
            return new double[]{state.getEntry(0), state.getEntry(1), normalizeAngle(state.getEntry(2))};
        }

        RealMatrix A = MatrixUtils.createRealIdentityMatrix(6);
        A.setEntry(0, 3, dt);
        A.setEntry(1, 4, dt);
        A.setEntry(2, 5, dt);

        RealVector predictedState = A.operate(state);
        return new double[]{predictedState.getEntry(0), predictedState.getEntry(1), normalizeAngle(predictedState.getEntry(2))};
    }
}