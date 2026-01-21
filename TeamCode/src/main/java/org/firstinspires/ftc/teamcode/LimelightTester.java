package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.SingularMatrixException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp
public class LimelightTester extends OpMode {

    private static final int LIMELIGHT_PIPELINE_INDEX = 4;
    // From https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html:
    // "Heading, or Yaw, is the measure of rotation about the Z axis, which points up toward the
    // ceiling."
    // "Rotation follows the traditional right-hand rule: with the thumb pointing along the
    // positive axis, the fingers curl in the direction of positive rotation."
    // Positive yaw therefore represents counterclockwise rotation from the point of view of someone
    // looking down on the robot.
    private static final boolean POSITIVE_IMU_ORIENTATION_IS_CLOCKWISE = false;
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
    private static double initialRobotYawRelativeToFieldInDegrees = 126;
    private static boolean initialRobotYawRelativeToFieldInDegreeHasBeenInitialized = false;
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final int goalAprilTagId;
    KalmanFilterXYThetaSensor poseFilter;
    private IMU imu;
    private Limelight3A limelight;

    public LimelightTester() {
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
        poseFilter = new KalmanFilterXYThetaSensor(0, 0,
                                                   initialRobotYawRelativeToFieldInDegrees
                , 0, 2, 2, 10);
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
                          initialRobotYawRelativeToFieldInDegrees + robotYawRelativeToStartInDegrees);
        telemetry.addData("Pitch reported by IMU",
                          robotYawPitchRollRelativeToStart.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll reported by IMU",
                          robotYawPitchRollRelativeToStart.getRoll(AngleUnit.DEGREES));
        telemetry.addLine("initialRobotYawRelativeToFieldInDegrees " + initialRobotYawRelativeToFieldInDegrees);
        telemetry.addLine("Setting Limelight robot orientation to " + initialRobotYawRelativeToFieldInDegrees + robotYawRelativeToStartInDegrees);
        limelight.updateRobotOrientation(initialRobotYawRelativeToFieldInDegrees + robotYawRelativeToStartInDegrees);
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
                //if (!initialRobotYawRelativeToFieldInDegreeHasBeenInitialized) {
                //    initialRobotYawRelativeToFieldInDegrees =
                //            s.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.DEGREES) - robotYawRelativeToStartInDegrees;
                //    initialRobotYawRelativeToFieldInDegreeHasBeenInitialized = true;
                //}
                limelight.updateRobotOrientation(initialRobotYawRelativeToFieldInDegrees + robotYawRelativeToStartInDegrees);
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
                if (s.getRobotPoseFieldSpace().getPosition().z < 0.2 || 0.4 < s.getRobotPoseFieldSpace().getPosition().z) {
                    telemetry.addLine("Robot pose field space Z is out of the acceptable range");
                    //continue;
                }
                try {
                    poseFilter.update(runtime.milliseconds() - llResult.getStaleness(),
                                      s.getRobotPoseFieldSpace().getPosition().x,
                                      s.getRobotPoseFieldSpace().getPosition().y,
                                      s.getRobotPoseFieldSpace().getOrientation().getYaw());
                } catch (SingularMatrixException e) {
                    telemetry.addLine("Caught SingularMatrixException");
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
            // milliseconds
        }
        telemetry.addData("limelight result is valid", isValid);
        double[] p = poseFilter.getPredictedPose(runtime.milliseconds());
        for (int i = 0; i < p.length; i++) {
            telemetry.addData("P" + i, p[i]);
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
