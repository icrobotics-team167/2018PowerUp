package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.y2018.Robot;

public class AutoUtil {

    private static final double MIN_MOTOR_MAGN = 0.275D;
    private static final double MIN_TURN_MAGN = 0.35D;

    public static void drive(Robot bot, double inches, double speed) {
        Data.pushState();

        bot.ahrs.reset();
        Vector4 vec = new Vector4(0, speed, 0, 0);
        double initialDist = bot.srcLidar.get();
        double deltaDist = Math.abs(inches);
        double oneMinusDownscale = MIN_MOTOR_MAGN / Math.abs(speed);
        double deltaFactor = (1D - oneMinusDownscale) / deltaDist;
        Source<Double> srcDelta = bot.srcLidar.map(Data.mapper(v -> 1D - Math.abs(v - initialDist) / deltaDist));
        bot.snkDrive.bind(Data.source(() -> vec)
                .inter(srcDelta, Data.inter(
                        (out, delta) -> out.y(speed * (delta * deltaFactor + oneMinusDownscale)).w(bot.ahrs.getAngle()))));
        bot.snkAutoProfile.bind(srcDelta);
        bot.srcLidar.get();
        Flow.waitUntil(() -> srcDelta.get() <= 0D);

        Data.popState();
    }

    public static void turn(Robot bot, double degrees, double speed) {
        Data.pushState();

        bot.ahrs.reset();
        double absSpeed = Math.abs(speed) * Math.signum(degrees);
        Vector4 vec = new Vector4(0, 0, absSpeed, 0);
        double initialAngle = bot.ahrs.getAngle();
        double deltaAngle = Math.abs(degrees);
        double oneMinusDownscale = MIN_TURN_MAGN / Math.abs(absSpeed);
        double deltaFactor = (1D - oneMinusDownscale) / deltaAngle;
        Source<Double> srcDelta = Data.source(() -> 1D - Math.abs(bot.ahrs.getAngle() - initialAngle) / deltaAngle);
        bot.snkDrive.bind(Data.source(() -> vec)
                .inter(srcDelta, Data.inter(
                        (out, delta) -> out.z(speed * (delta * deltaFactor + oneMinusDownscale)))));
        bot.snkAutoProfile.bind(srcDelta);
        Flow.waitUntil(() -> srcDelta.get() <= 0.05D);

        Data.popState();
    }

}
