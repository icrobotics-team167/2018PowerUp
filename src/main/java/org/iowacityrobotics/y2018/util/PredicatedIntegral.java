package org.iowacityrobotics.y2018.util;

import org.iowacityrobotics.roboed.data.inter.Interpolator;
import org.iowacityrobotics.roboed.util.math.Maths;

public class PredicatedIntegral extends Interpolator<Boolean, Double, Double> {

    private final double initial, lower, upper;
    private double accum;

    public PredicatedIntegral(double initial, double lower, double upper) {
        this.accum = this.initial = initial;
        this.lower = lower;
        this.upper = upper;
    }

    @Override
    public void reset(boolean temp) {
        if (!temp) accum = initial;
    }

    @Override
    public Double apply(Boolean predicate, Double differential) {
        if (predicate) accum = Maths.clamp(accum + differential, lower, upper);
        return accum;
    }

}
