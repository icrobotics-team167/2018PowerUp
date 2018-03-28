package org.iowacityrobotics.y2018.subsystem;

public enum RampState {

    AUTO(0.05D, 0.135D),
    UP(0.015D, 0.1D),
    DOWN(0.2D, 0.285D);

    public final double a, b;

    RampState(double a, double b) {
        this.a = a;
        this.b = b;
    }

}
