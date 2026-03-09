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

    public void log(java.lang.String format, java.lang.Object... args) {
        // Only the first nine entries display on the driver station screen
        //telemetry.log().add(format, args);
        RobotLog.vv(tag, format, args);
    }

    public void log2(java.lang.String format, java.lang.Object... args) {
        telemetry.log().add(format, args);
        RobotLog.vv(tag, format, args);
    }

    public Telemetry.Item addItem(java.lang.String caption,
                                  java.lang.Object value) {
        Telemetry.Item item = telemetry.addData(caption, value);
        RobotLog.vv(tag, caption + ": " + value);
        return item;
    }

    public Telemetry.Item updateItem(Telemetry.Item item,
                                  java.lang.Object value) {
        item.setValue(value);
        RobotLog.vv(tag, item.getCaption() + ": " + value);
        return item;
    }

    public void addData(java.lang.String caption,
                                  java.lang.Object value) {
        telemetry.addData(caption, value);
        RobotLog.vv(tag, caption + ": " + value);
    }

    public void addData(java.lang.String caption,
                           java.lang.String format,
                           java.lang.Object... args) {
        telemetry.addData(caption, format, args);
        RobotLog.vv(tag, caption + ": " + String.format(format, args));
    }

    public void addLine(java.lang.String lineCaption) {
        telemetry.addLine(lineCaption);
        RobotLog.vv(tag, lineCaption);
    }

}
