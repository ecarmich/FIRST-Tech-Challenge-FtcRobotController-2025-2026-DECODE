package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * TeleOp mode that lets the driver turn the bot to a specified heading.
 */
@TeleOp
public class AimingTester extends OpMode {
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
    private LazyImu lazyImu;
    private double power = 0.0;

    /*
     * Each press of the D-pad should change the power by one increment. This tracks whether we
     * already registered this button press.
     */
    private boolean dPadPressed = false;

    private long lastElapsedTimeNanoSeconds;

    private int numLoopCalls = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

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
        lastElapsedTimeNanoSeconds = runtime.nanoseconds();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
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
        rotateCounterClockwise(power);

        // Display motor performance statistics
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Power", power);
        telemetry.addData("Velocity", leftFront.getVelocity());
        telemetry.addData("Current", leftFront.getCurrent(CurrentUnit.MILLIAMPS) + " milliamps");
        telemetry.addData("Yaw", "%.4f (%.4f degrees)", lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
                lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch", "%.4f (%.4f degrees)", lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS)
                , lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll", "%.4f (%.4f degrees)", lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS)
                , lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        telemetry.addData("Number of calls to loop()", "%,d", numLoopCalls);
        telemetry.addData("Time since last call to loop()", "%,d", runtime.nanoseconds() - lastElapsedTimeNanoSeconds);
        lastElapsedTimeNanoSeconds = runtime.nanoseconds();
        numLoopCalls++;
        telemetry.addData("Avg nanoseconds between loop() calls", "%,d", runtime.nanoseconds() / numLoopCalls);
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
