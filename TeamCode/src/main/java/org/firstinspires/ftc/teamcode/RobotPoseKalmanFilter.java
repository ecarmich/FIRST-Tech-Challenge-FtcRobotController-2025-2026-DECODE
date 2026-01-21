package org.firstinspires.ftc.teamcode;

import org.apache.commons.math3.filter.*;
import org.apache.commons.math3.linear.*;

public class RobotPoseKalmanFilter {

    // Indexes for clarity
    private static final int X = 0, Y = 1, THETA = 2;
    private static final int VX = 3, VY = 4, OMEGA = 5;
    private static final int AX = 6, AY = 7, ALPHA = 8;
    private final KalmanFilter filter;
    private final DynamicMeasurementModel measureModel;
    private final RealMatrix A; // State Transition Matrix
    private double lastTimestamp;

    public RobotPoseKalmanFilter(double initX, double initY, double initTheta, double startTime) {
        // 1. Initialize State: [x, y, theta, vx, vy, omega, ax, ay, alpha]
        RealVector initialState = new ArrayRealVector(9);
        initialState.setEntry(X, initX);
        initialState.setEntry(Y, initY);
        initialState.setEntry(THETA, initTheta);

        // 2. Initial Uncertainty (P)
        RealMatrix P = MatrixUtils.createRealIdentityMatrix(9).scalarMultiply(1.0);

        // 3. State Transition (A) - Will be updated with dt every step
        A = MatrixUtils.createRealIdentityMatrix(9);

        // 4. Process Noise (Q) - Uncertainty in our "Constant Acceleration" physics model
        // We trust the physics less for acceleration (indices 6-8) than position
        RealMatrix Q = MatrixUtils.createRealIdentityMatrix(9);
        Q.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(0.01).getData(), 0, 0); // Pos noise
        Q.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(0.1).getData(), 3, 3);  // Vel noise
        Q.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(1.0).getData(), 6, 6);  // Accel noise

        ProcessModel pm = new DefaultProcessModel(A, null, Q, initialState, P);

        // 5. Initialize with dummy measurement model
        measureModel = new DynamicMeasurementModel();
        filter = new KalmanFilter(pm, measureModel);

        this.lastTimestamp = startTime;
    }

    /**
     * Handle Position/Orientation Sensor (Measures indices 0, 1, 2)
     */
    public synchronized void updatePose(double t, double x, double y, double theta, double stdX, double stdY, double stdTheta) {
        double dt = t - lastTimestamp;
        if (dt < 0) {
            return; // Skip out-of-order measurements
        }

        predict(dt);

        // H maps state [9x1] to measurement [3x1]
        RealMatrix H = MatrixUtils.createRealMatrix(3, 9);
        H.setEntry(0, X, 1.0);
        H.setEntry(1, Y, 1.0);
        H.setEntry(2, THETA, 1.0);

        // R is built from the standard deviations reported by the sensor
        RealMatrix R = MatrixUtils.createRealDiagonalMatrix(new double[]{Math.pow(stdX, 2), Math.pow(stdY, 2), Math.pow(stdTheta, 2)});

        measureModel.setMatrices(H, R);

        // Note: For production code, you must normalize the theta error here
        // to handle the 359->1 degree wrap-around.
        filter.correct(new ArrayRealVector(new double[]{x, y, theta}));

        lastTimestamp = t;
    }

    /**
     * Handle Velocity Sensor (Measures indices 3, 4, 5)
     */
    public synchronized void updateVelocity(double t, double vx, double vy, double omega, double stdV, double stdOmega) {
        double dt = t - lastTimestamp;
        if (dt < 0) return;

        predict(dt);

        // H maps state [9x1] to measurement [3x1]
        RealMatrix H = MatrixUtils.createRealMatrix(3, 9);
        H.setEntry(0, VX, 1.0);
        H.setEntry(1, VY, 1.0);
        H.setEntry(2, OMEGA, 1.0);

        // R uses fixed/estimated std dev since this sensor doesn't report it dynamically
        RealMatrix R = MatrixUtils.createRealDiagonalMatrix(new double[]{Math.pow(stdV, 2), Math.pow(stdV, 2), Math.pow(stdOmega, 2)});

        measureModel.setMatrices(H, R);
        filter.correct(new ArrayRealVector(new double[]{vx, vy, omega}));

        lastTimestamp = t;
    }

    private void predict(double dt) {
        if (dt <= 1e-9) return;

        // Update Physics Matrix A (p = p + v*dt + 0.5*a*dt^2)
        // We do this for X, Y, and Theta simultaneously

        // Position += Velocity * dt + 0.5 * Accel * dt^2
        A.setEntry(X, VX, dt);
        A.setEntry(X, AX, 0.5 * dt * dt);
        A.setEntry(Y, VY, dt);
        A.setEntry(Y, AY, 0.5 * dt * dt);
        A.setEntry(THETA, OMEGA, dt);
        A.setEntry(THETA, ALPHA, 0.5 * dt * dt);

        // Velocity += Accel * dt
        A.setEntry(VX, AX, dt);
        A.setEntry(VY, AY, dt);
        A.setEntry(OMEGA, ALPHA, dt);

        filter.predict();
    }

    public double[] getPose() {
        double[] s = filter.getStateEstimation();
        return new double[]{s[X], s[Y], s[THETA]};
    }

    // Helper to swap H and R matrices dynamically
    private static class DynamicMeasurementModel implements MeasurementModel {
        private RealMatrix H = MatrixUtils.createRealMatrix(3, 9);
        private RealMatrix R = MatrixUtils.createRealIdentityMatrix(3);

        public void setMatrices(RealMatrix H, RealMatrix R) {
            this.H = H;
            this.R = R;
        }

        @Override
        public RealMatrix getMeasurementMatrix() {
            return H;
        }

        @Override
        public RealMatrix getMeasurementNoise() {
            return R;
        }
    }
}
