package org.iowacityrobotics.y2018.util;

public enum SpeedLevel {

    NIL(),
    FAST(1D / 3D),
    FASTER(2D / 3D),
    FASTEST(1D);

    public final double factor;
    public final boolean invalid;

    SpeedLevel(double factor) {
        this.factor = factor;
        this.invalid = false;
    }

    SpeedLevel() {
        this.factor = 0D;
        this.invalid = true;
    }

}
