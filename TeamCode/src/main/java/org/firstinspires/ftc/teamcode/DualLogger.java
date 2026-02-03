package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Logger that logs messages to the driver station and to the robot controller's log file.
 * <p>
 * To view the robot controller's log file, use a USB cable to connect a computer to the control
 * hub's USB-C port, open Android Studio on the computer, and click on the Logcat icon in the lower
 * left.
 */
public class DualLogger {
    private final Telemetry telemetry;
    private final String tag;

    DualLogger(Telemetry telemetry, String tag) {
        this.telemetry = telemetry;
        this.tag = tag;
    }

    public Telemetry.Item addData(java.lang.String caption,
                           java.lang.String format,
                           java.lang.Object... args) {
        Telemetry.Item item = telemetry.addData(caption, format, args);
        RobotLog.vv(tag, caption + ": " + String.format(format, args));
        return item;
    }
}
