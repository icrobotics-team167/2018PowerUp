package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.util.math.Maths;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Linear motion profile feedback controller
 */
public class LiftController {

    private static final double TOLERANCE = 1200D / 27500D;
    private static final double ERROR = 600D / 27500D;
    private static final double SPEED = 0.5D;

    private final Sink<Double> lift;
    private final double[] setpoint;
    private final AtomicBoolean met;
    private final Source<Double> source;

    public LiftController(Source<Double> enc, Sink<Double> lift) {
        this.lift = lift;
        this.setpoint = new double[2];
        this.met = new AtomicBoolean(false);
        this.source = enc.map(Data.mapper(feedback -> {
            if (met.get()) {
                if (feedback < setpoint[0] || feedback > setpoint[2]) met.set(false);
                return 0D;
            } else {
                double diff = setpoint[1] - feedback;
                if (Math.abs(diff) <= ERROR) {
                    met.set(true);
                    return 0D;
                } else {
                    return Math.signum(diff) * SPEED;
                }
            }
        }));
    }

    public void bind() {
        lift.bind(source);
    }

    public void unbind() {
        lift.bind(null);
    }

    public void set(double pos) {
        setpoint[0] = Maths.clamp(pos - TOLERANCE, 0D, 1D);
        setpoint[1] = Maths.clamp(pos, 0D, 1D);
        setpoint[2] = Maths.clamp(pos + TOLERANCE, 0D, 1D);
        met.set(false);
    }

}
