package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {
    // Gains
    private final double kp; // Proportional
    private final double ki; // Integral
    private final double kd; // Derivative

    private final Telemetry telemetry;

    private double targetSetpoint;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTimestamp = 0;

    // Output limits to prevent "Integral Windup"
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    public PIDController(double kp, double ki, double kd, Telemetry telemetry) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.telemetry = telemetry;
    }

    public void setSetpoint(double setpoint) {
        this.targetSetpoint = setpoint;
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public double calculate(double currentMeasurement, long currentTimestamp) {
        // Calculate time delta (dt)
        double dt = (currentTimestamp - lastTimestamp) / 1000000000.0;
        if (dt <= 0) {
            dt = 0.001; // Avoid division by zero
        }
        telemetry.addData("dt", dt);

        // 1. Error calculation
        double error = targetSetpoint - currentMeasurement;
        telemetry.addData("error", error);

        // 2. Proportional term
        double pTerm = kp * error;

        // If we're so far away from the target that we need to turn at max speed, we don't need the
        // other PID terms.
        if (Math.abs(pTerm) >= 1) {
            reset(currentTimestamp);
            return pTerm;
        }

        // 3. Integral term (accumulation of error over time)
        integralSum += error * dt;
        double iTerm = ki * integralSum;

        // 4. Derivative term (rate of change of error)
        double dTerm = kd * (error - lastError) / dt;

        telemetry.addData("pTerm", pTerm);
        telemetry.addData("iTerm", iTerm);
        telemetry.addData("dTerm", dTerm);
        // Total Output
        double output = pTerm + iTerm + dTerm;

        // Save states for next iteration
        lastError = error;
        lastTimestamp = currentTimestamp;

        // Clamp output to limits
        return Math.max(minOutput, Math.min(maxOutput, output));
    }

    public void reset(long nanoseconds) {
        integralSum = 0;
        lastError = 0;
        lastTimestamp = nanoseconds;
    }
}