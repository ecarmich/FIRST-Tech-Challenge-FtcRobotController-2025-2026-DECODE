package org.firstinspires.ftc.teamcode;

public class VariablePeriodPIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private double setpoint;
    private double previousError;
    private double accumulatedError;

    public VariablePeriodPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.previousError = 0;
        this.accumulatedError = 0;
    }

    public double calculate(double measurement, double deltaTime) {
        // Calculate the current error
        double error = setpoint - measurement;

        // Proportional term
        double proportionalTerm = kP * error;

        // Integral term (scaled by deltaTime)
        accumulatedError += error * deltaTime;
        double integralTerm = kI * accumulatedError;

        // Derivative term (rate of change of error, scaled by deltaTime)
        double derivativeTerm = kD * (error - previousError) / deltaTime;

        // Update previous error for the next iteration
        previousError = error;

        // Calculate the total output
        return proportionalTerm + integralTerm + derivativeTerm;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void reset() {
        this.accumulatedError = 0;
        this.previousError = 0;
    }
}