package slab;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.util.logging.Logs;

public class FakeLift {

    private static final int MAX_POS = 30000;
    private static final double VEL_FACTOR = 30D;
    private static final double GRAV_FACTOR = 6D;

    public final Source<Boolean> sourceLimit;
    public final Sink<Double> sink;

    private int pos;
    private long lastSim;
    private double[] vel;

    public FakeLift() {
        this.pos = 0;
        this.lastSim = -1L;
        this.vel = new double[] {0};
        this.sourceLimit = Data.source(() -> get() > 0);
        this.sink = Data.sink(v -> vel[0] = (v == null ? 0D : v));
    }

    public int get() {
        long time = System.nanoTime();
        if (lastSim > 0L) {
            double elapsed = (time - lastSim) / 1e6D;
            double rate = vel[0] * VEL_FACTOR - GRAV_FACTOR;
            Logs.info(String.format("%d += %.2f u/ms * %.2f ms", pos, rate, elapsed));
            pos = Math.max(Math.min((int)Math.floor(pos + elapsed * rate), MAX_POS), 0);
        }
        lastSim = time;
        return pos;
    }

}
