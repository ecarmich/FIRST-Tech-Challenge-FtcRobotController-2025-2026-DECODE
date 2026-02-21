package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.time.Instant;
import java.util.concurrent.TimeUnit;

@TeleOp
public class AutoMove extends OpMode {

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
    private static final double INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES = 180;

    private static final int LIMELIGHT_PIPELINE_INDEX = 4;

    private final ElapsedTime runtime = new ElapsedTime();

    private final DualLogger dualLogger = new DualLogger(telemetry, "CoachEric");

    private final int goalAprilTagId;
    VariablePeriodPIDController xPid;
    VariablePeriodPIDController yPid;
    VariablePeriodPIDController thetaPid;
    long lastTime;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private Angle imuYaw;
    private double pinpointXMeters;
    private double pinpointYMeters;
    private double pinpointYawRadians;
    private final PinpointDriftKalmanFilter filterDriftX = new PinpointDriftKalmanFilter(0.1, 0.1);
    private final PinpointDriftKalmanFilter filterDriftY = new PinpointDriftKalmanFilter(0.1, 0.1);
    private final PinpointDriftKalmanFilter filterDriftTheta = new PinpointDriftKalmanFilter(0.1, 0.1);
    private double estimatedDriftXMeters;
    private double estimatedDriftYMeters;
    private double estimatedDriftThetaRadians;
    private MecanumDrive mecanumDrive;

    private long lastLimelightReadNanoseconds;

    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;

    public AutoMove() {
        goalAprilTagId = 24;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dualLogger.addData("Status", "Initializing");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        initializeImu();

        initializeLimelight();

        initializePinpoint();

        // Initialize the motors
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);

        // Tell the driver that initialization is complete.
        dualLogger.addData("Status", "Initialized");
    }

    private void initializeImu() {
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
    }

    private void initializePinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        boolean pinpointInitializedSuccessfully = pinpoint.initialize();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Moving the robot forward should increase the estimated X position, and moving it left
        // should increase the estimated Y position
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // For localization purposes, the center of the robot is the point where the diagonal
        // connecting the centers of the left front and back right wheels crosses the diagonal
        // connecting the centers of the right front and back left wheels.
        pinpoint.setOffsets(3.75, -7.5, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        dualLogger.addLine("Pinpoint initialized successfully: " + pinpointInitializedSuccessfully);
    }

    private void initializeLimelight() {
        // Initialize the Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE_INDEX);
        limelight.start();
        dualLogger.addLine("Limelight initialized. Pipeline index = " + LIMELIGHT_PIPELINE_INDEX);
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
        dualLogger.addData("Status", "Start requested");
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        dualLogger.addData("Entering loop(). System.nanoTime()", System.nanoTime());
        dualLogger.addData("Entering loop(). System.nanoTime()", System.nanoTime());
        dualLogger.addData("Entering loop(). System.currentTimeMillis()", System.currentTimeMillis());
        dualLogger.addData("Entering loop(). System.nanoTime()", System.nanoTime());
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            dualLogger.addData("Entering loop(). Instant.now()", Instant.now());
        }
        // Nanoseconds from power on to runtime.reset()?
        dualLogger.addData("Entering loop(). runtime.startTimeNanoseconds()", runtime.startTimeNanoseconds());
        // Nanoseconds since power on?
        dualLogger.addData("Entering loop(). runtime.now(TimeUnit.NANOSECONDS)", runtime.now(TimeUnit.NANOSECONDS));
        readImu();
        readPinpoint();
        // Must follow call to readImu() because it uses the yaw reported by the IMU.
        readLimelight();

        dualLogger.addData("Estimated x", pinpointXMeters - estimatedDriftXMeters);
        dualLogger.addData("Estimated y", pinpointYMeters - estimatedDriftYMeters);
        dualLogger.addData("Estimated theta", pinpointYawRadians - estimatedDriftThetaRadians);

        move();
    }

    private void readImu() {
        dualLogger.addLine("Entering readImu()");
        // This call takes 6-8 milliseconds
        imuYaw = Angle.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        //YawPitchRollAngles robotYawPitchRollRelativeToStart = imu.getRobotYawPitchRollAngles();
        //double robotYawRelativeToStartInDegrees =
        //        robotYawPitchRollRelativeToStart.getYaw(AngleUnit.DEGREES);
        //dualLogger.addData("Yaw reported by IMU", robotYawRelativeToStartInDegrees);
        dualLogger.addData("Yaw reported by IMU", imuYaw);
        //dualLogger.addData("IMU Yaw relative to field",
                //INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
        //dualLogger.addData("Pitch reported by IMU",
        //        robotYawPitchRollRelativeToStart.getPitch(AngleUnit.DEGREES));
        //dualLogger.addData("Roll reported by IMU",
        //        robotYawPitchRollRelativeToStart.getRoll(AngleUnit.DEGREES));
        //dualLogger.addLine("initialRobotYawRelativeToFieldInDegrees " + INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES);
        //dualLogger.addLine("Setting Limelight robot orientation to " + INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
    }

    private void readPinpoint() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        //dualLogger.addData("Pinpoint pose", pose);
        pinpointXMeters = pose.getX(DistanceUnit.METER);
        dualLogger.addData("pinpointXMeters", pinpointXMeters);
        pinpointYMeters = pose.getY(DistanceUnit.METER);
        dualLogger.addData("pinpointYMeters", pinpointYMeters);
        pinpointYawRadians = pose.getHeading(AngleUnit.RADIANS);
        dualLogger.addData("pinpointYawRadians", pinpointYawRadians);
    }

    private boolean readLimelight() {
        long currentTimeNanoseconds = runtime.nanoseconds();
        long timeSinceLastLimelightReadNanoseconds = currentTimeNanoseconds - lastLimelightReadNanoseconds;

        limelight.updateRobotOrientation(INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + Math.toDegrees(pinpointYawRadians));
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid() || llResult.getPipelineIndex() != LIMELIGHT_PIPELINE_INDEX) {
            dualLogger.addData("estimatedDriftX", "");
            dualLogger.addData("estimatedDriftY", "");
            dualLogger.addData("estimatedDriftTheta", "");
            return false;
        }
        boolean foundGoalTag = false;
        for (LLResultTypes.FiducialResult fr : llResult.getFiducialResults()) {
            if (fr.getFiducialId() == 20 || fr.getFiducialId() == 24) {
                foundGoalTag = true;
                break;
            }
        }
        if (!foundGoalTag) {
            dualLogger.addData("estimatedDriftX", "");
            dualLogger.addData("estimatedDriftY", "");
            dualLogger.addData("estimatedDriftTheta", "");
            return false;
        }

        // System time in milliseconds?
        dualLogger.addData("llResult.getTimestamp()", llResult.getTimestamp());
        // Age of the result in milliseconds, according to https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming#8-is-the-data-fresh
        // I've verified that if you wait 500 milliseconds and then re-query this, it grows by 500
        dualLogger.addData("llResult.getStaleness()", llResult.getStaleness());
        // This always returns 37.619998931884766
        dualLogger.addData("llResult.getCaptureLatency()", llResult.getCaptureLatency());
        dualLogger.addData("llResult.getParseLatency()", llResult.getParseLatency());
        dualLogger.addData("llResult.getTargetingLatency()", llResult.getTargetingLatency());
        // Epoch time in nanoseconds
        dualLogger.addData("llResult.getControlHubTimeStampNanos()", llResult.getControlHubTimeStampNanos());
        /*
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 500) {

        }
        dualLogger.addData("llResult.getTimestamp() after waiting 500 milliseconds", llResult.getTimestamp());
        dualLogger.addData("llResult.getStaleness() after waiting 500 milliseconds", llResult.getStaleness());
        // This always returns 37.619998931884766
        dualLogger.addData("llResult.getCaptureLatency() after waiting 500 milliseconds", llResult.getCaptureLatency());
        dualLogger.addData("llResult.getParseLatency() after waiting 500 milliseconds", llResult.getParseLatency());
        dualLogger.addData("llResult.getTargetingLatency() after waiting 500 milliseconds", llResult.getTargetingLatency());
        // Epoch time in nanoseconds
        dualLogger.addData("llResult.getControlHubTimeStampNanos() after waiting 500 milliseconds", llResult.getControlHubTimeStampNanos());
*/

        // Distances are in meters, rotations are in degrees. This is centered on the origin.
        // Zero yaw points along the x axis (i.e., towards the audience). Positive yaw indicates
        // counterclockwise rotation from the point of view of someone looking down on the field
        // (i.e., same as the unit circle).
        dualLogger.addData("getBotpose()", llResult.getBotpose().toString());

        Pose3D botPose = llResult.getBotpose_MT2();
        dualLogger.addData("getBotpose_MT2()", botPose.toString());

        double limelightXMeters = botPose.getPosition().x;
        dualLogger.addData("limelightXMeters", limelightXMeters);
        dualLogger.addData("limelightXStdDev", llResult.getStddevMt2()[0]);
        filterDriftX.addMeasurements(timeSinceLastLimelightReadNanoseconds, pinpointXMeters, limelightXMeters, llResult.getStddevMt2()[0]);
        estimatedDriftXMeters = filterDriftX.getEstimatedDrift();
        dualLogger.addData("estimatedDriftXMeters", estimatedDriftXMeters);
        double limelightYMeters = botPose.getPosition().y;
        dualLogger.addData("limelightYMeters", limelightYMeters);
        dualLogger.addData("limelightYStdDev", llResult.getStddevMt2()[1]);
        filterDriftY.addMeasurements(timeSinceLastLimelightReadNanoseconds, pinpointYMeters, limelightYMeters, llResult.getStddevMt2()[1]);
        estimatedDriftYMeters = filterDriftY.getEstimatedDrift();
        dualLogger.addData("estimatedDriftYMeters", estimatedDriftYMeters);
        double limelightYawRadians = botPose.getOrientation().getYaw(AngleUnit.RADIANS);
        dualLogger.addData("limelightYawRadians", limelightYawRadians);
        dualLogger.addData("limelightYawStdDev", llResult.getStddevMt2()[5]);
        filterDriftTheta.addMeasurements(timeSinceLastLimelightReadNanoseconds, pinpointYawRadians, limelightYawRadians, llResult.getStddevMt2()[5]);
        estimatedDriftThetaRadians = filterDriftTheta.getEstimatedDrift();
        dualLogger.addData("estimatedDriftThetaRadians", estimatedDriftThetaRadians);

        lastLimelightReadNanoseconds = currentTimeNanoseconds;
        return true;
    }

    private void move() {
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
            double currentX = pinpointXMeters - estimatedDriftXMeters;
            double currentY = pinpointYMeters - estimatedDriftYMeters;
            double currentTheta = pinpointYawRadians - estimatedDriftThetaRadians;

            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            double deltaTheta = targetTheta - currentTheta;
            dualLogger.addData("deltaX", deltaX);
            dualLogger.addData("deltaY", deltaY);
            dualLogger.addData("deltaTheta", deltaTheta);
            double movementAngleRelativeToFieldInRadians = Math.atan2(deltaY, deltaX);
            double movementAngleRelativeToFieldInDegrees = Math.toDegrees(movementAngleRelativeToFieldInRadians);
            dualLogger.addData("movementAngleRelativeToFieldInDegrees", movementAngleRelativeToFieldInDegrees);
            double movementAngleRelativeToRobotInDegrees = movementAngleRelativeToFieldInDegrees - currentTheta;
            dualLogger.addData("movementAngleRelativeToRobotInDegrees", movementAngleRelativeToRobotInDegrees);
            double movementDistanceMeters = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            dualLogger.addData("movementDistanceMeters", movementDistanceMeters);

            long now = runtime.nanoseconds();
            double deltaTime = (now - lastTime) / 1_000_000_000.0;
            lastTime = now;

            double xOutput = xPid.calculate(currentX, deltaTime);
            double yOutput = yPid.calculate(currentY, deltaTime);
            double thetaOutput = thetaPid.calculate(currentTheta, deltaTime);
            dualLogger.addData("xOutput", xOutput);
            dualLogger.addData("yOutput", yOutput);
            dualLogger.addData("thetaOutput", thetaOutput);

            double forwardRelativeToBot = movementDistanceMeters * Math.cos(Math.toRadians(movementAngleRelativeToRobotInDegrees));
            double strafeRelativeToBot = movementDistanceMeters * Math.sin(Math.toRadians(movementAngleRelativeToRobotInDegrees));
            double turnRelativeToBot = deltaTheta / 20;

        } else {
            xPid = null;
            yPid = null;
            thetaPid = null;
            mecanumDrive.moveRelativeToRobot(0, 0, 0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        dualLogger.addData("Status", "Stop requested");
        limelight.stop();
    }

}