package org.firstinspires.ftc.teamcode;

/**
 * A 1D Kalman filter specifically tuned for fusing goBILDA Pinpoint and Limelight data.
 * <p>
 * This filter treats "drift" as the hidden state. It models drift as a random walk
 * where uncertainty grows over time and is corrected by visual updates.
 * <p>
 * Each instance of this class can provide estimates for one dimension (x, y, or theta). Measurement
 * units are up to the user, but must be consistent; i.e., p, driftVariance, pinpointMeasurement,
 * limelightMeasurement, and limelightVariance should all be in meters, or all be in millimeters, or
 * all be in radians, etc.
 * <p>
 * Based on <a href="https://medium.com/@vikramaditya.nishant/how-to-make-a-zero-drift-ftc-localizer-with-kalman-filters-911807e0916d">...</a>
 */
public class PinpointDriftKalmanFilter {

    // Process noise (Q). Higher values mean the drift changes more rapidly over time. We model the
    // drift's change over time as a random walk. This is our estimate of its variance.
    private final double driftVariance;

    // These two variables together constitute the state of the filter.

    // Estimate of the difference between the Pinpoint's value and the true value. We don't
    // initialize this because the starting value is zero; it only becomes non-zero when we get a
    // Pinpoint reading and a Limelight reading that differ.
    private double estimatedDrift;

    // The variance of the estimate; in a multidimensional Kalman filter, this would be the error
    // covariance matrix, or P. We don't have any actual uncertainty to start with, but the Kalman
    // filter math requires a non-zero value.
    private double uncertainty; // Uncertainty (P)

    public PinpointDriftKalmanFilter(double initialUncertainty, double driftVariance) {
        this.uncertainty = initialUncertainty;
        this.driftVariance = driftVariance;
    }

    /**
     * Updates the filter state using a single combined Predict/Update cycle.
     * <p>
     * Many Kalman filter implementations have separate predict() and update() methods, where
     * predict() updates the state based on elapsed time, and update() updates the state based on
     * new measurements, but in our model time elapsing only increases uncertainty, so we combine
     * the two methods into one to make it easier for the user.
     * <p>
     *
     * @param deltaTime            Time since the last update.
     * @param pinpointMeasurement  The raw value from the Pinpoint sensor.
     * @param limelightMeasurement The "Ground Truth" value from the Limelight.
     * @param limelightVariance    The "Measurement Noise" (R). This is reported by the Limelight.
     *                             Higher values mean you should trust the Limelight less.
     *                             </p>
     */
    public void addMeasurements(double deltaTime, double pinpointMeasurement, double limelightMeasurement, double limelightVariance) {
        // -- Prediction step --

        // As time passes, we naturally become less certain about our drift estimate.
        uncertainty += driftVariance * deltaTime;

        // -- Measurement update step --

        double observedDrift = pinpointMeasurement - limelightMeasurement;
        double innovation = observedDrift - estimatedDrift;

        // Kalman Gain: How much do we trust this new data vs our current estimate?
        double kalmanGain = uncertainty / (uncertainty + limelightVariance);

        // Adjust the estimate based on how much we trust the new observation
        estimatedDrift += kalmanGain * innovation;

        // Update the error covariance (uncertainty decreases because we have new info)
        uncertainty = (1 - kalmanGain) * uncertainty;
    }

    public double getEstimatedDrift() {
        return estimatedDrift;
    }
}
