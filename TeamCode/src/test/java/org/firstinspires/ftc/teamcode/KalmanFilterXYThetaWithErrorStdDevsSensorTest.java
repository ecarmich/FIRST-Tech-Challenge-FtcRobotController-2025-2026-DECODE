package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static com.google.common.truth.Truth.assertThat;

public class KalmanFilterXYThetaWithErrorStdDevsSensorTest {

    @Test
    public void noUpdate() {
        KalmanFilterXYThetaWithErrorStdDevsSensor kalmanFilterXYThetaWithErrorStdDevsSensor = new KalmanFilterXYThetaWithErrorStdDevsSensor(0, 0, 0, 0);
        assertThat(kalmanFilterXYThetaWithErrorStdDevsSensor.getX()).isEqualTo(0);
        assertThat(kalmanFilterXYThetaWithErrorStdDevsSensor.getY()).isEqualTo(0);
    }

    @Test
    public void update() {
        KalmanFilterXYThetaWithErrorStdDevsSensor kalmanFilterXYThetaWithErrorStdDevsSensor = new KalmanFilterXYThetaWithErrorStdDevsSensor(0, 0, 0, 0);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(1000, 1, 2, 0.3, 0.1, 0.1, 0.1);
        assertThat(kalmanFilterXYThetaWithErrorStdDevsSensor.getX()).isWithin(0.00001).of(0.9999);
        assertThat(kalmanFilterXYThetaWithErrorStdDevsSensor.getY()).isWithin(0.00001).of(1.9998);
        assertThat(kalmanFilterXYThetaWithErrorStdDevsSensor.getTheta()).isWithin(0.00001).of(0.29997);
    }

    @Test
    public void predict() {
        KalmanFilterXYThetaWithErrorStdDevsSensor kalmanFilterXYThetaWithErrorStdDevsSensor = new KalmanFilterXYThetaWithErrorStdDevsSensor(0, 0, 0, 0);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(1000, 1, 2, 0.3, 0.1, 0.1, 0.1);
        double[] prediction = kalmanFilterXYThetaWithErrorStdDevsSensor.getPredictedPose(2000);
        assertThat(prediction[0]).isWithin(0.00001).of(1.9898);
        assertThat(prediction[1]).isWithin(0.00001).of(3.9796);
        assertThat(prediction[2]).isWithin(0.00001).of(0.59694);
    }

    @Test
    public void predict2() {
        KalmanFilterXYThetaWithErrorStdDevsSensor kalmanFilterXYThetaWithErrorStdDevsSensor = new KalmanFilterXYThetaWithErrorStdDevsSensor(0, 0, 0, 0);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(1000, 1, 2, 0.3, 0.1, 0.1, 0.1);
        double[] prediction = kalmanFilterXYThetaWithErrorStdDevsSensor.getPredictedPose(1500);
        assertThat(prediction[0]).isWithin(0.00001).of(1.9898);
        assertThat(prediction[1]).isWithin(0.00001).of(3.9796);
        assertThat(prediction[2]).isWithin(0.00001).of(0.59694);
    }

    @Test
    public void predict3() {
        KalmanFilterXYThetaWithErrorStdDevsSensor kalmanFilterXYThetaWithErrorStdDevsSensor = new KalmanFilterXYThetaWithErrorStdDevsSensor(0, 0, 0, 0);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(1000, 1, 2, 0.3, 10, 10, 10);
        assertThat(kalmanFilterXYThetaWithErrorStdDevsSensor.getX()).isWithin(0.00001).of(0.9999);
        double[] prediction = kalmanFilterXYThetaWithErrorStdDevsSensor.getPredictedPose(2000);
        assertThat(prediction[0]).isWithin(0.00001).of(1.9898);
        assertThat(prediction[1]).isWithin(0.00001).of(3.9796);
        assertThat(prediction[2]).isWithin(0.00001).of(0.59694);
    }

    @Test
    public void predict4() {
        KalmanFilterXYThetaWithErrorStdDevsSensor kalmanFilterXYThetaWithErrorStdDevsSensor = new KalmanFilterXYThetaWithErrorStdDevsSensor(0, 0, 0, 0);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(1000, 1.05, 0, 0, 0.1, 0.1, 0.1);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(2000, 1.95, 0, 0, 0.1, 0.1, 0.1);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(3000, 3.1, 0, 0, 0.1, 0.1, 0.1);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(4000, 4.2, 0, 0, 0.1, 0.1, 0.1);
        kalmanFilterXYThetaWithErrorStdDevsSensor.update(5000, 4.9, 0, 0, 0.1, 0.1, 0.1);
        //(robotFilter.getX()).isWithin(0.00001).of(0.9999);
        double[] prediction = kalmanFilterXYThetaWithErrorStdDevsSensor.getPredictedPose(6000);
        assertThat(prediction[0]).isWithin(0.00001).of(1.9898);
        assertThat(prediction[1]).isWithin(0.00001).of(3.9796);
        assertThat(prediction[2]).isWithin(0.00001).of(0.59694);
    }
}