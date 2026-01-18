package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TeleOp mode that lets the driver turn the bot to a specified heading.
 */
@TeleOp
public class BrakingTester extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    private IMU imu;

    /*
     * Each press of the D-pad should change the power by one increment. This tracks whether we
     * already registered this button press.
     */
    private boolean dPadPressed = false;

    private double power = 0.0;

    private long brakingStartTimeNanoSeconds;
    private Angle brakingStartHeading;
    private double brakingStartPower;
    private double brakingStartVelocity;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        //leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        //rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        //leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        //rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection)).get();
        imu.resetYaw();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Use the D-pad (Directional Pad)"
                + " to change the power:\n"
                + "  Up: add 0.1\n"
                + "  Down: subtract 0.1\n"
                + "  Left: add 0.01\n"
                + "  Right: subtract 0.01");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        telemetry.addData("Status", "Start requested");
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Power", power);
        telemetry.addData("Right back velocity", rightBack.getVelocity(AngleUnit.RADIANS));
        if (brakingStartTimeNanoSeconds > 0) {
            Angle currentHeading = getYawInRadians();
            Angle brakingDistance = currentHeading.subtract(brakingStartHeading);
            while (brakingStartPower>0 && brakingDistance.degrees < 0) {
                brakingDistance = Angle.fromDegrees(brakingDistance.degrees + 360);
            }
            telemetry.addData("Braking start power", brakingStartPower);
            telemetry.addData("Braking start velocity", brakingStartVelocity);
            telemetry.addData("Braking start heading", brakingStartHeading);
            telemetry.addData("Current heading", currentHeading);
            telemetry.addData("Braking distance", brakingDistance);
            if (gamepad1.y) {
                brakingStartTimeNanoSeconds = 0;
            }
            return;
        }
        if (gamepad1.x) {
            brakingStartTimeNanoSeconds = runtime.startTimeNanoseconds();
            brakingStartHeading = Angle.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            brakingStartVelocity = rightBack.getVelocity(AngleUnit.RADIANS);
            brakingStartPower = power;
            power = 0.01;
        } else {
            updatePower();
        }
        rotateCounterClockwise(power);
    }

    private Angle getYawInRadians() {
        return Angle.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
    private void updatePower() {
        if (dPadPressed) {
            // We previously registered a button press
            if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                // None of the D-pad buttons are currently being pressed, so reset dPadPressed so that the next button press will register
                dPadPressed = false;
            }
            // Exit without updating the power
            return;
        }
        if (gamepad1.dpad_up) {
            dPadPressed = true;
            power += 0.1;
        }
        if (gamepad1.dpad_down) {
            dPadPressed = true;
            power -= 0.1;
        }
        if (gamepad1.dpad_left) {
            dPadPressed = true;
            power += 0.01;
        }
        if (gamepad1.dpad_right) {
            dPadPressed = true;
            power -= 0.01;
        }
        if (power > 1.0) {
            power = 1;
        }
        if (power < -1) {
            power = -1;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stop requested");
    }

    private void rotateCounterClockwise(double power) {
        if (power > 1.0) {
            power = 1;
        } else if (power < -1) {
            power = -1;
        }
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }
}
