package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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
    private static final double INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES = 126;

    private static final int LIMELIGHT_PIPELINE_INDEX = 4;

    private final ElapsedTime runtime = new ElapsedTime();
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
    private double estimatedDriftX;
    private double estimatedDriftY;
    private double estimatedDriftTheta;
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
        telemetry.addData("Status", "Initializing");

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
        telemetry.addData("Status", "Initialized");
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
        pinpoint.setOffsets(3.125, 7.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        telemetry.addLine("Pinpoint initialized successfully: " + pinpointInitializedSuccessfully);
    }

    private void initializeLimelight() {
        // Initialize the Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE_INDEX);
        limelight.start();
        telemetry.addLine("Limelight initialized. Pipeline index = " + LIMELIGHT_PIPELINE_INDEX);
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
        readImu();
        readPinpoint();
        // Must follow call to readImu() because it uses the yaw reported by the IMU.
        readLimelight();

        telemetry.addData("Estimated x", pinpointXMeters - estimatedDriftX);
        telemetry.addData("Estimated y", pinpointYMeters - estimatedDriftY);
        telemetry.addData("Estimated theta", pinpointYawRadians - estimatedDriftTheta);

        move();
    }

    private void readImu() {
        imuYaw = Angle.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        YawPitchRollAngles robotYawPitchRollRelativeToStart = imu.getRobotYawPitchRollAngles();
        double robotYawRelativeToStartInDegrees =
                robotYawPitchRollRelativeToStart.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw reported by IMU", robotYawRelativeToStartInDegrees);
        telemetry.addData("IMU Yaw relative to field",
                INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
        //telemetry.addData("Pitch reported by IMU",
        //        robotYawPitchRollRelativeToStart.getPitch(AngleUnit.DEGREES));
        //telemetry.addData("Roll reported by IMU",
        //        robotYawPitchRollRelativeToStart.getRoll(AngleUnit.DEGREES));
        //telemetry.addLine("initialRobotYawRelativeToFieldInDegrees " + INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES);
        telemetry.addLine("Setting Limelight robot orientation to " + INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + robotYawRelativeToStartInDegrees);
    }

    private void readPinpoint() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pinpoint pose", pose);
        pinpointXMeters = pose.getX(DistanceUnit.METER);
        telemetry.addData("pinpointXMeters", pinpointXMeters);
        pinpointYMeters = pose.getY(DistanceUnit.METER);
        telemetry.addData("pinpointYMeters", pinpointYMeters);
        pinpointYawRadians = pose.getHeading(AngleUnit.RADIANS);
        telemetry.addData("pinpointYawRadians", pinpointYawRadians);
    }

    private boolean readLimelight() {
        long currentTimeNanoseconds = runtime.nanoseconds();
        long timeSinceLastLimelightReadNanoseconds = currentTimeNanoseconds - lastLimelightReadNanoseconds;

        limelight.updateRobotOrientation(INITIAL_ROBOT_YAW_RELATIVE_TO_FIELD_IN_DEGREES + imuYaw.degrees);
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid() || llResult.getPipelineIndex() != LIMELIGHT_PIPELINE_INDEX) {
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
            return false;
        }

        // Distances are in meters, rotations are in degrees. This is centered on the origin.
        // Zero yaw points along the x axis (i.e., towards the audience). Positive yaw indicates
        // counterclockwise rotation from the point of view of someone looking down on the field
        // (i.e., same as the unit circle).
        Pose3D botPose = llResult.getBotpose_MT2();
        telemetry.addData("BotPose", botPose.toString());

        double limelightXMeters = botPose.getPosition().x;
        filterDriftX.addMeasurements(timeSinceLastLimelightReadNanoseconds, pinpointXMeters, limelightXMeters, llResult.getStddevMt2()[0]);
        estimatedDriftX = filterDriftX.getEstimatedDrift();
        telemetry.addData("estimatedDriftX", estimatedDriftX);
        double limelightYMeters = botPose.getPosition().y;
        filterDriftY.addMeasurements(timeSinceLastLimelightReadNanoseconds, pinpointYMeters, limelightYMeters, llResult.getStddevMt2()[1]);
        estimatedDriftY = filterDriftY.getEstimatedDrift();
        telemetry.addData("estimatedDriftY", estimatedDriftY);
        double limelightYawRadians = botPose.getOrientation().getYaw(AngleUnit.RADIANS);
        filterDriftTheta.addMeasurements(timeSinceLastLimelightReadNanoseconds, pinpointYawRadians, limelightYawRadians, llResult.getStddevMt2()[5]);
        estimatedDriftTheta = filterDriftTheta.getEstimatedDrift();
        telemetry.addData("estimatedDriftTheta", estimatedDriftTheta);

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
            double currentX = pinpointXMeters - estimatedDriftX;
            double currentY = pinpointYMeters - estimatedDriftY;
            double currentTheta = pinpointYawRadians - estimatedDriftTheta;

            double deltaX = targetX - currentX;
            double deltaY = targetY - currentY;
            double deltaTheta = targetTheta - currentTheta;
            telemetry.addData("deltaX", deltaX);
            telemetry.addData("deltaY", deltaY);
            telemetry.addData("deltaTheta", deltaTheta);
            double movementAngleRelativeToFieldInRadians = Math.atan2(deltaY, deltaX);
            double movementAngleRelativeToFieldInDegrees = Math.toDegrees(movementAngleRelativeToFieldInRadians);
            telemetry.addData("movementAngleRelativeToFieldInDegrees", movementAngleRelativeToFieldInDegrees);
            double movementAngleRelativeToRobotInDegrees = movementAngleRelativeToFieldInDegrees - currentTheta;
            telemetry.addData("movementAngleRelativeToRobotInDegrees", movementAngleRelativeToRobotInDegrees);
            double movementDistanceMeters = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            telemetry.addData("movementDistanceMeters", movementDistanceMeters);

            long now = runtime.nanoseconds();
            double deltaTime = (now - lastTime) / 1_000_000_000.0;
            lastTime = now;

            double xOutput = xPid.calculate(currentX, deltaTime);
            double yOutput = yPid.calculate(currentY, deltaTime);
            double thetaOutput = thetaPid.calculate(currentTheta, deltaTime);
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