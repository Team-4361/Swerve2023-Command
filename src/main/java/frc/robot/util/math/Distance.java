package frc.robot.util.math;

public class Distance {
    private double meterValue;
    private DistanceUnit unit;

    public Distance(double value, DistanceUnit unit) {
        this.meterValue = convert(value, unit, DistanceUnit.METERS);
        this.unit = unit;
    }

    public Distance setDistance(double value, DistanceUnit unit) {
        this.meterValue = convert(value, unit, DistanceUnit.METERS);
        this.unit = unit;
        return this;
    }

    public DistanceUnit getDistanceUnit() {
        return this.unit;
    }

    public double getValue(DistanceUnit unit) {
        return convert(meterValue, DistanceUnit.METERS, unit);
    }

    public double getValue() {
        return convert(meterValue, DistanceUnit.METERS, this.unit);
    }

    public static Distance fromValue(double value, DistanceUnit unit) {
        return new Distance(value, unit);
    }

    public static double convert(double value, DistanceUnit oldUnit, DistanceUnit newUnit) {
        switch (oldUnit) {
            case FEET:
               switch (newUnit) {
                   case FEET: return value; // no change
                   case INCHES: return value * 12;
                   case METERS: return value / 3.281;
                   case CENTIMETERS: return value / 30.48;
                   default: return value;
               }
            case INCHES:
                switch (newUnit) {
                    case FEET: return value / 12;
                    case INCHES: return value; // no change
                    case METERS: return value / 39.37;
                    case CENTIMETERS: return value * 2.54;
                    default: return value;
                }
            case METERS:
                switch (newUnit) {
                    case FEET: return value * 3.281;
                    case INCHES: return value * 39.37;
                    case METERS: return value; // no change
                    case CENTIMETERS: return value * 100;
                    default: return value;
                }
            case CENTIMETERS:
                switch (newUnit) {
                    case FEET: return value / 30.48;
                    case INCHES: return value / 2.54;
                    case METERS: return value / 100;
                    case CENTIMETERS: return value; // no change
                    default: return value;
                }
            default: return value;
        }
    }
}
