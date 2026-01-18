package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import java.util.Locale;

public class Angle {
    enum Unit {
        DEGREES,
        RADIANS
    }

    Unit unit;
    double degrees;
    double radians;

    private Angle(double value, Unit unit) {
        this.unit = unit;
        if (unit == Unit.DEGREES) {
            degrees = value;
            radians = Math.toRadians(degrees);
        } else {
            radians = value;
            degrees = Math.toDegrees(radians);
        }
    }

    public static Angle fromDegrees(double degrees) {
        return new Angle(degrees, Unit.DEGREES);
    }

    public static Angle fromRadians(double radians) {
        return new Angle(radians, Unit.RADIANS);
    }

    public Angle subtract(Angle other) {
        if (this.unit == Unit.DEGREES) {
            return fromDegrees(this.degrees - other.degrees);
        } else {
            return fromRadians(this.radians - other.radians);
        }
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "%.2f radians (%.2f degrees)", radians, degrees);
    }
}
