package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp
public class AutoMove extends OpMode {

    private static final int LIMELIGHT_PIPELINE_INDEX = 4;
    // From https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html:
    // "Heading, or Yaw, is the measure of rotation about the Z axis, which points up toward the
    // ceiling."
    // "Rotation follows the traditional right-hand rule: with the thumb pointing along the
    // positive axis, the fingers curl in the direction of positive rotation."
    // Positive yaw therefore represents counterclockwise rotation from the point of view of someone
    // looking down on the robot.

    // The FTC field coordinate system is explained here:
    // https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
    //
    // The origin is in the center of the field, touching the top of the mat.
    // The z axis points to the ceiling, the x axis points to the audience,
    // and the y axis points away from the red alliance area.
    //
    // The initial orientation of the robot relative to the field, in degrees.
    // Zero points along the x axis (i.e., towards the audience). Positive values indicate clockwise
    // rotation, and negative values indicate counterclockwise rotation.
    //
    // 126 degrees is pointing towards the red goal, perpendicular to the red
    // AprilTag (i.e., the orientation of a bot that's facing the red goal and
    // touching it).
    private static final double INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES = 126;

    private final ElapsedTime runtime = new ElapsedTime();
    private final int goalAprilTagId;
    private IMU imu;
    private Limelight3A limelight;

    private MecanumDrive mecanumDrive;

    VariablePeriodPIDController xPid;
    VariablePeriodPIDController yPid;
    VariablePeriodPIDController thetaPid;
    long lastTime;

    public AutoMove() {
        goalAprilTagId = 24;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        // See https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
        // On this robot, the control hub is mounted with the logo facing up and the USB ports
        // facing forward.
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        boolean imuInitializationSucceeded =
                imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        imu.resetYaw();
        telemetry.addData("IMU initialized successfully", imuInitializationSucceeded);

        // Initialize the Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE_INDEX);
        limelight.start();

        // Initialize the motors
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        YawPitchRollAngles robotYawPitchRollRelativeToStart = imu.getRobotYawPitchRollAngles();
        double robotYawRelativeToStartInDegrees =
                robotYawPitchRollRelativeToStart.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw reported by IMU", robotYawRelativeToStartInDegrees);
        telemetry.addData("IMU Yaw relative to field",
                          INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
        telemetry.addData("Pitch reported by IMU",
                          robotYawPitchRollRelativeToStart.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll reported by IMU",
                          robotYawPitchRollRelativeToStart.getRoll(AngleUnit.DEGREES));
        telemetry.addLine("initialRobotYawRelativeToFieldInDegrees " + INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES);
        telemetry.addLine("Setting Limelight robot orientation to " + INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
        limelight.updateRobotOrientation(INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
        LLResult llResult = limelight.getLatestResult();
        boolean isValid = false;
        if (llResult != null && llResult.isValid()) {
            isValid = true;
            if (llResult.getPipelineIndex() != LIMELIGHT_PIPELINE_INDEX) {
                telemetry.addData("Error", "Pipeline index: expected %d, got %d",
                                  LIMELIGHT_PIPELINE_INDEX, llResult.getPipelineIndex());
                return;
            }
            // Distances are in meters, rotations are in degrees. This is centered on the origin.
            // Zero yaw points along the x axis (i.e., towards the audience). Positive yaw indicates
            // counterclockwise rotation from the point of view of someone looking down on the field
            // (i.e., same as the unit circle).
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.addData("Pipeline index", llResult.getPipelineIndex());
            telemetry.addData("IMU orientation", robotYawRelativeToStartInDegrees);
            telemetry.addData("Elapsed time", runtime.time());
            for (LLResultTypes.FiducialResult s : llResult.getFiducialResults()) {
                telemetry.addData("Fiducial ID", s.getFiducialId());
                if (s.getFiducialId() != goalAprilTagId) {
                    continue;
                }
                telemetry.addData("Robot pose field space", s.getRobotPoseFieldSpace());
                limelight.updateRobotOrientation(INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
                for (List<Double> corner : s.getTargetCorners()) {
                    StringBuilder line = new StringBuilder();
                    for (Double d : corner) {
                        if (line.length() > 0) {
                            line.append(", ");
                        }
                        line.append(d);
                    }
                    telemetry.addLine(line.toString());
                }
                telemetry.addData("Camera pose target space", s.getCameraPoseTargetSpace());
                telemetry.addData("Target pose camera space", s.getTargetPoseCameraSpace());
            }
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            double distance = (29.5 - 13.75) / Math.tan(Math.toRadians(llResult.getTy()));
            telemetry.addData("Computed distance", distance);
            telemetry.addData("Angle to target",
                              llResult.getTx() + robotYawRelativeToStartInDegrees);
            //telemetry.addData("Target XNC", llResult.getTxNC());
            //telemetry.addData("Target Area", llResult.getTa());
            //telemetry.addData("Latency", llResult.getStaleness()); // Latency of the result in

            if (gamepad1.a) {
                double targetX = 0;
                double targetY = 0;
                double targetTheta = 135;
                if (xPid == null) {
                    xPid = new VariablePeriodPIDController(1, 1, 1);
                    xPid.setSetpoint(targetX);
                    yPid = new VariablePeriodPIDController(1, 1, 1);
                    yPid.setSetpoint(targetY);
                    thetaPid = new VariablePeriodPIDController(1, 1, 1);
                    thetaPid.setSetpoint(targetTheta);
                    lastTime = runtime.nanoseconds();
                }
                double deltaX = targetX - botPose.getPosition().x;
                double deltaY = targetY - botPose.getPosition().y;
                double deltaTheta = targetTheta - botPose.getOrientation().getYaw();
                telemetry.addData("deltaX", deltaX);
                telemetry.addData("deltaY", deltaY);
                telemetry.addData("deltaTheta", deltaTheta);
                double movementAngleRelativeToFieldInRadians = Math.atan2(deltaY, deltaX);
                double movementAngleRelativeToFieldInDegrees = Math.toDegrees(movementAngleRelativeToFieldInRadians);
                telemetry.addData("movementAngleRelativeToFieldInDegrees", movementAngleRelativeToFieldInDegrees);
                double movementAngleRelativeToRobotInDegrees = movementAngleRelativeToFieldInDegrees - botPose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("movementAngleRelativeToRobotInDegrees", movementAngleRelativeToRobotInDegrees);
                double movementDistanceMeters = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                telemetry.addData("movementDistanceMeters", movementDistanceMeters);

                long now = runtime.nanoseconds();
                double deltaTime = (now - lastTime) / 1_000_000_000.0;
                lastTime = now;

                double xOutput = xPid.calculate(botPose.getPosition().x, deltaTime);
                double yOutput = yPid.calculate(botPose.getPosition().y, deltaTime);
                double thetaOutput = thetaPid.calculate(botPose.getOrientation().getYaw(), deltaTime);
                telemetry.addData("xOutput", xOutput);
                telemetry.addData("yOutput", yOutput);
                telemetry.addData("thetaOutput", thetaOutput);

                double forwardRelativeToBot = movementDistanceMeters * Math.cos(Math.toRadians(movementAngleRelativeToRobotInDegrees));
                double strafeRelativeToBot = movementDistanceMeters * Math.sin(Math.toRadians(movementAngleRelativeToRobotInDegrees));
                double turnRelativeToBot = deltaTheta / 20;

            } else {
                xPid = null;
                yPid = null;
                thetaPid = null;
                mecanumDrive.moveRelativeToRobot(0, 0, 0);
            }
        } else {
            xPid = null;
            yPid = null;
            thetaPid = null;
            mecanumDrive.moveRelativeToRobot(0, 0, 0);
        }
        telemetry.addData("limelight result is valid", isValid);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stop requested");
        limelight.stop();
    }
}
