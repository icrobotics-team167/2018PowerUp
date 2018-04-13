package org.iowacityrobotics.y2018.subsystem;

public enum RampState {

    AUTO(0.0175D, 0.0525D),
    UP(0.015D, 0.05D),
    DOWN(0.26D, 0.27D);

    public final double a, b;

    RampState(double a, double b) {
        this.a = a;
        this.b = b;
    }

}
