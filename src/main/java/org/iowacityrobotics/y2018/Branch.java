package org.iowacityrobotics.y2018;

public class Branch<T> {

    private final T tVal, fVal;

    public Branch(T tVal, T fVal) {
        this.tVal = tVal;
        this.fVal = fVal;
    }

    public T get(boolean dir) {
        return dir ? tVal : fVal;
    }

}
