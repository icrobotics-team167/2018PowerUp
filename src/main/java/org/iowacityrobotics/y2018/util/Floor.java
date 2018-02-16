package org.iowacityrobotics.y2018.util;

public enum Floor { // TODO Implement elevator presets
    SOMEWHERE(1),
    SOMEWHERE_ELSE(2);

    public final int count;

    Floor(int count) {
        this.count = count;
    }
}
