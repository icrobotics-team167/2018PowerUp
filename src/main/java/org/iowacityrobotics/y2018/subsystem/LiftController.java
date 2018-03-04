package org.iowacityrobotics.y2018.subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.math.Maths;
import org.iowacityrobotics.y2018.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Linear motion profile feedback controller
 */
public class LiftController {

    private static final double TOLERANCE = 1200D / 27500D;
    private static final double ERROR = 600D / 27500D;
    private static final double SPEED = 0.375D;

    private final Sink<Double> lift;
    private final double[] setpoint;
    private final AtomicBoolean met;
    private final Source<Double> source;

    public LiftController(Source<Double> enc, Sink<Double> lift) {
        this.lift = lift;
        this.setpoint = new double[3];
        this.met = new AtomicBoolean(false);
        this.source = enc.map(Data.mapper(feedback -> {
            SmartDashboard.putNumber("Lower", setpoint[0]);
            SmartDashboard.putNumber("Target", setpoint[1]);
            SmartDashboard.putNumber("Upper", setpoint[2]);
            if (met.get()) {
                SmartDashboard.putNumber("Error", 0);
                if (feedback < setpoint[0] || feedback > setpoint[2]) met.set(false);
                return 0D;
            } else {
                double diff = setpoint[1] - feedback;
                SmartDashboard.putNumber("Error", diff);
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

    public void blockUntilMet(Robot bot) {
        Data.pushState();
        Data.reset(true);
        bind();
        bot.snkLiftEnc.bind(bot.srcLiftEnc);
        Flow.waitUntil(met::get);
        Data.popState();
    }

    public void setBlocking(double pos, Robot bot) {
        set(pos);
        blockUntilMet(bot);
    }

}
