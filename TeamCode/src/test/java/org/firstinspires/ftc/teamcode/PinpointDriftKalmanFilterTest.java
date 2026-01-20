package org.firstinspires.ftc.teamcode;

import static com.google.common.truth.Truth.assertThat;

import org.junit.jupiter.api.Test;

class PinpointDriftKalmanFilterTest {
    @Test
    public void initialEstimateIsZero() {
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        assertThat(filter.getEstimatedDrift()).isEqualTo(0);
    }

    @Test
    public void pinpointMatchesLimelightExactly() {
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        for (int i = 1; i <= 100; i++) {
            filter.addMeasurements(1, i, i, 1);
            assertThat(filter.getEstimatedDrift()).isEqualTo(0);
        }
    }

    @Test
    public void pinpointDiffersFromLimelight() {
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(1, 1.01, 1, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.005024875621890552);
    }

    @Test
    public void pinpointDiffersFromLimelightLongerTimeBetweenMeasurements() {
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(10, 1.01, 1, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.009091734786557682);
    }

    @Test
    public void pinpointDiffersFromLimelightHigherStartingP() {
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.1, 1);
        filter.addMeasurements(1, 1.01, 1, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.005238095238095243);
    }

    @Test
    public void pinpointDiffersFromLimelightLowerDriftVariance() {
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, .1);
        filter.addMeasurements(1, 1.01, 1, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(9.909909909909917E-4);
    }

    @Test
    public void lowerLimelightVarianceIncreasesTheEstimatedDrift() {
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(1, 1.01, 1, 0.1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.009099099099099107);
    }

    @Test
    public void pinpointAlwaysOnePercentTooHigh() {
        // The difference between the Pinpoint's estimate and the Limelight's estimate grows over
        // time
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(1, 1.01, 1, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.005024875621890552);
        filter.addMeasurements(1, 2.02, 2, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.014015904572564627);
        filter.addMeasurements(1, 3.03, 3, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.023853211009174195);
        filter.addMeasurements(1, 4.04, 4, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.03382636655948551);
        filter.addMeasurements(1, 5.05, 5, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.043821328866554876);
    }

    @Test
    public void relativeErrorIsTheOnlyThingThatMatters() {
        // The Pinpoint and Limelight measurements are different from the ones in
        // pinpointAlwaysOnePercentTooHigh(), but the differences between them are the same
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(1, 0.01, 0, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.005024875621890552);
        filter.addMeasurements(1, 0.02, 0, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.014015904572564627);
        filter.addMeasurements(1, 0.03, 0, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.023853211009174195);
        filter.addMeasurements(1, 0.04, 0, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.03382636655948551);
        filter.addMeasurements(1, 0.05, 0, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.043821328866554876);
    }

    @Test
    public void pinpointDriftsAndLimelightHasNoise() {
        // The actual position remains zero.
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(1, .02, 0.01, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.005024875621890552);
        filter.addMeasurements(1, 0.01, -.01, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.014015904572564627);
        filter.addMeasurements(1, 0.02, -.05, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.04847094801223241);
        filter.addMeasurements(1, -0.01, .001, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.011738380590470623);
        filter.addMeasurements(1, 0, 0.012, 1);
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(-0.0029314349525404806);
    }

    @Test
    public void limelightErrorConstant() {
        // Pinpoint always returns 5. Limelight always returns 0. Estimated drift increases over
        // time.
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(1, 5, 0, 10);
        double lastDriftEstimate = 0.45867393278837426;
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(lastDriftEstimate);
        for (int i = 2; i <= 100; i++) {
            filter.addMeasurements(1, 5, 0, 10);
            assertThat(filter.getEstimatedDrift()).isGreaterThan(lastDriftEstimate);
            lastDriftEstimate = filter.getEstimatedDrift();
        }
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(4.999999999999819);
    }

    @Test
    public void limelightErrorAlternates() {
        // Pinpoint always returns zero. Limelight alternately returns 5 and -5.
        // Estimated drift alternates.
        PinpointDriftKalmanFilter filter = new PinpointDriftKalmanFilter(0.01, 1);
        filter.addMeasurements(1, 0, 5, 10);
        double lastError = -0.45867393278837426;
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(lastError);
        for (int i = 2; i <= 100; i++) {
            if (i % 2 == 1) {
                filter.addMeasurements(1, 0, 5, 10);
                assertThat(filter.getEstimatedDrift()).isLessThan(0);
            } else {
                filter.addMeasurements(1, 0, -5, 10);
                assertThat(filter.getEstimatedDrift()).isGreaterThan(0);
            }
        }
        assertThat(filter.getEstimatedDrift()).isWithin(0.00001).of(0.7808688094430258);
    }
}