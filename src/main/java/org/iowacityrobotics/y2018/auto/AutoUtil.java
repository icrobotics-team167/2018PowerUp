package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.y2018.Robot;

public class AutoUtil {

    private static final double MIN_MOTOR_MAGN = 0.35D;
    private static final double MIN_TURN_MAGN = 0.50D;

    public static void driveWithTimeout(Robot bot, double inches, double speed, long timeout) {
        Data.pushState();

        bot.ahrs.reset();
        Vector4 vec = new Vector4(0, -speed, 0, 0);
        double initialDist = bot.srcLidarF.get();
        double deltaDist = Math.abs(inches);
        double oneMinusDownscale = MIN_MOTOR_MAGN / Math.abs(speed);
        double deltaFactor = (1D - oneMinusDownscale) / deltaDist;
        Source<Double> srcDelta = bot.srcLidarF.map(Data.mapper(v -> 1D - Math.abs(v - initialDist) / deltaDist));
        bot.snkDrive.bind(Data.source(() -> vec)
                .inter(srcDelta, Data.inter(
                        (out, delta) -> out.y(-speed * (delta * deltaFactor + oneMinusDownscale)).w(bot.ahrs.getAngle()))));
        bot.snkAutoProfile.bind(srcDelta);
        bot.srcLidarF.get();
        long initialTime = System.currentTimeMillis();
        Flow.waitUntil(() -> srcDelta.get() <= 0D || System.currentTimeMillis() - initialTime > timeout);

        Data.popState();
    }

    public static void drive(Robot bot, double inches, double speed) {
        Data.pushState();

        bot.ahrs.reset();
        Vector4 vec = new Vector4(0, -speed, 0, 0);
        double initialDist = bot.srcLidarF.get();
        double deltaDist = Math.abs(inches);
        double oneMinusDownscale = MIN_MOTOR_MAGN / Math.abs(speed);
        double deltaFactor = (1D - oneMinusDownscale) / deltaDist;
        Source<Double> srcDelta = bot.srcLidarF.map(Data.mapper(v -> 1D - Math.abs(v - initialDist) / deltaDist));
        bot.snkDrive.bind(Data.source(() -> vec)
                .inter(srcDelta, Data.inter(
                        (out, delta) -> out.y(-speed * (delta * deltaFactor + oneMinusDownscale)).w(bot.ahrs.getAngle()))));
        bot.snkAutoProfile.bind(srcDelta);
        bot.srcLidarF.get();
        Flow.waitUntil(() -> srcDelta.get() <= 0D);

        Data.popState();
    }

    public static void strafe(Robot bot, double inches, double speed) {
        Data.pushState();

        bot.ahrs.reset();
        double absSpeed = Math.abs(speed) * Math.signum(inches);
        Vector4 vec = new Vector4(absSpeed, 0, 0, 0);
        double initialDist = bot.srcLidarS.get();
        double deltaDist = Math.abs(inches);
        double oneMinusDownscale = MIN_MOTOR_MAGN / Math.abs(speed);
        double deltaFactor = (1D - oneMinusDownscale) / deltaDist;
        Source<Double> srcDelta = bot.srcLidarS.map(Data.mapper(v -> 1D - Math.abs(v - initialDist) / deltaDist));
        bot.snkDrive.bind(Data.source(() -> vec)
                .inter(srcDelta, Data.inter(
                        (out, delta) -> out.x(absSpeed * (delta * deltaFactor + oneMinusDownscale)).w(bot.ahrs.getAngle()))));
        bot.snkAutoProfile.bind(srcDelta);
        bot.srcLidarS.get();
        Flow.waitUntil(() -> srcDelta.get() <= 0D);

        Data.popState();
    }

    public static void strafeBlind(Robot bot, long ms, double speed) {
        Data.pushState();

        bot.ahrs.reset();
        Vector4 vec = new Vector4(speed, 0, 0, 0);
        bot.snkDrive.bind(Data.source(() -> vec));
        Flow.waitFor(ms);

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
        bot.snkIntake.bind(Data.source(() -> -0.675D));
        Flow.waitUntil(() -> srcDelta.get() <= 2.5D);

        Data.popState();
    }

    public static void skillshot(Robot bot, boolean theQuickness) {
        Data.pushState();
        double speed = theQuickness ? 0.5D : 0.3275D;
        bot.snkIntake.bind(Data.source(() -> speed));
        Flow.waitFor(1500);
        Data.popState();
    }


}
