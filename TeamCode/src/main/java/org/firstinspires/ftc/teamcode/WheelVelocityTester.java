package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class WheelVelocityTester extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    private double velocity = 0.0;

    /*
     * Each press of the D-pad should change the velocity by one increment. This tracks whether we
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
                + " to change the velocity:\n"
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
            // Exit without updating the velocity
            return;
        }
        if (gamepad1.dpad_up) {
            dPadPressed = true;
            velocity += 100;
        }
        if (gamepad1.dpad_down) {
            dPadPressed = true;
            velocity -= 100;
        }
        if (gamepad1.dpad_left) {
            dPadPressed = true;
            velocity += 10;
        }
        if (gamepad1.dpad_right) {
            dPadPressed = true;
            velocity -= 10;
        }
        if (velocity > 3000) {
            velocity = 3000;
        }
        if (velocity < -3000) {
            velocity = -3000;
        }
        if (gamepad1.y) {
            velocity = 1500;
        } else if (gamepad1.a) {
            velocity = -1500;
        }
        leftFront.setVelocity(velocity);
        rightFront.setVelocity(velocity);
        leftBack.setVelocity(velocity);
        rightBack.setVelocity(velocity);

        // Display motor performance statistics
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Velocity", velocity);

        telemetry.addData("Left Front Velocity ", leftFront.getVelocity());
        telemetry.addData("Right Front Velocity", rightFront.getVelocity());
        telemetry.addData("Left Back Velocity  ", leftBack.getVelocity());
        telemetry.addData("Right Back Velocity ", rightBack.getVelocity());

        telemetry.addData("Left Front Power ", leftFront.getPower());
        telemetry.addData("Right Front Power", rightFront.getPower());
        telemetry.addData("Left Back Power  ", leftBack.getPower());
        telemetry.addData("Right Back Power ", rightBack.getPower());

        telemetry.addData("Left Front Current ", leftFront.getCurrent(CurrentUnit.MILLIAMPS) + " milliamps");
        telemetry.addData("Right Front Current", rightFront.getCurrent(CurrentUnit.MILLIAMPS) + " milliamps");
        telemetry.addData("Left Back Current  ", leftBack.getCurrent(CurrentUnit.MILLIAMPS) + " milliamps");
        telemetry.addData("Right Back Current ", rightBack.getCurrent(CurrentUnit.MILLIAMPS) + " milliamps");
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stop requested");
    }
}
