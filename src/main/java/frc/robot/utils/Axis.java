package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;

public class Axis {
    public DoubleSupplier supplier;
    public SlewRateLimiter slr;
    public boolean square;

    public Axis(DoubleSupplier supplier, boolean square, double slewRate) {
        this.supplier = supplier;
        this.slr = new SlewRateLimiter(slewRate);
        this.slr.reset(0);
        this.square = square;
    }
    public Axis(DoubleSupplier supplier, double slewRate) {
        this(supplier, false, slewRate);
    }

    public Axis(DoubleSupplier supplier) {
        this(supplier, 10);
    }

    public double get(boolean applySRL) {
        double val = MathUtil.applyDeadband(supplier.getAsDouble(), Constants.stickDeadband);
        return applySRL ? slr.calculate(val) : val;
    }

    public double get() {
        return get(false);
    }
}
