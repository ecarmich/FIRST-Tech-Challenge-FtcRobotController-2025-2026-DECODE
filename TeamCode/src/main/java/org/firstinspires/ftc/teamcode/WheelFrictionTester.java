package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * TeleOp mode that lets the driver use the D-pad to set the wheel power in increments of 0.01
 */
@TeleOp
public class WheelFrictionTester extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private double power = 0.0;

    /*
     * Each press of the D-pad should change the power by one increment. This tracks whether we
     * already registered this button press.
     */
    private boolean dPadPressed = false;

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
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        // Display motor performance statistics
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Power", power);
        telemetry.addData("Velocity", leftFront.getVelocity());
        telemetry.addData("Current", leftFront.getCurrent(CurrentUnit.MILLIAMPS) + " milliamps");
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stop requested");
    }
}
