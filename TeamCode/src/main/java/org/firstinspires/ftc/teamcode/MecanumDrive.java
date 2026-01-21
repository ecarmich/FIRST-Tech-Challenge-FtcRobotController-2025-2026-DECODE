package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {

    private final Telemetry telemetry;
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx leftBackMotor;
    private final DcMotorEx rightBackMotor;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Move the robot in the given directions relative to the field.
     *
     * @param x Amount to move along the x axis
     * @param y Amount to move along the y axis
     * @param counterClockwise Amount to rotate counterclockwise
     * @param robotYawRelativeToField The amount the robot is rotated relative to the field, in
     *                                degrees. Zero yaw points along the x axis. Positive values
     *                                indicate counterclockwise rotation from the point of view of
     *                                someone looking down on the field.
     * <p>
     * If the robot is at a 90 degree angle relative to the field, movement of (1, 0) relative
     * to the field would be (0, -1) relative to the robot.
     * 90, (1, 0), (0, -1)
     * 45, (1, 0), (sqrt(1/2), -sqrt(1/2))
     */
    public void moveRelativeToField(double x, double y, double counterClockwise, double robotYawRelativeToField) {
        double forward = x * Math.cos(Math.toRadians(robotYawRelativeToField)) + y * Math.sin(Math.toRadians(robotYawRelativeToField));
        double strafeLeft = -x * Math.sin(Math.toRadians(robotYawRelativeToField)) + y * Math.cos(Math.toRadians(robotYawRelativeToField));
        moveRelativeToRobot(forward, strafeLeft, counterClockwise);
    }

    /**
     * Moves the robot in the given directions.
     *
     * @param forward Speed to move forward. Range: [-1, 1]
     * @param strafeLeft Speed to strafe right. Range: [-1, 1]
     * @param counterClockwise Speed to turn counterclockwise. Range: [-1, 1]
     * <p>
     * Parameter values represent desired motor powers. Motor powers are restricted to [-1, 1], so
     * if the parameter values result in motor powers outside those bounds, the motor powers will be
     * scaled accordingly.
     */
    public void moveRelativeToRobot(double forward, double strafeLeft, double counterClockwise) {
        double leftFrontPower = forward - strafeLeft - counterClockwise;
        double rightFrontPower = forward + strafeLeft + counterClockwise;
        double leftBackPower = forward + strafeLeft - counterClockwise;
        double rightBackPower = forward - strafeLeft + counterClockwise;

        double denominator = getDenominator(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        leftFrontPower /= denominator;
        rightFrontPower /= denominator;
        leftBackPower /= denominator;
        rightBackPower /= denominator;

        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightBackPower", rightBackPower);

        //leftFront.setPower(leftFrontPower);
        //rightFront.setPower(rightFrontPower);
        //leftBack.setPower(leftBackPower);
        //rightBack.setPower(rightBackPower);
    }

    private double getDenominator(double... motorPowers) {
        double denominator = 1;
        for (double motorPower : motorPowers) {
            denominator = Math.max(denominator, motorPower);
        }
        return denominator;
    }
}
