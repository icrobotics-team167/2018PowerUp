package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.y2018.Robot;

public class AutoUtil {

    private static final double MIN_MOTOR_MAGN = 0.35D;
    private static final double MIN_TURN_MAGN = 0.255D;
    private static final double COR_TURN_MAGN = -0.08D; // negative magnitude of correctional turning
    private static final double COR_TURN_THRESH = 1.75D; // angular error, in degrees, to start correction at

    public static void driveWithTimeout(Robot bot, double inches, double speed, long timeout) {
        Data.pushState();

        double realSpeed = -Math.abs(speed) * Math.signum(inches);
        Vector4 vec = new Vector4(0, realSpeed, 0, 0);
        double initialDist = bot.srcLidarF.get();
        double deltaDist = Math.abs(inches);
        double errMargin = inches / 5000D;
        double oneMinusDownscale = MIN_MOTOR_MAGN / Math.abs(speed);
        double deltaFactor = (1D - oneMinusDownscale) / deltaDist;
        Source<Double> srcDelta = bot.srcLidarF.map(Data.mapper(v -> 1D - Math.abs(v - initialDist) / deltaDist));

        bot.ahrs.reset();
        double initialAngle = bot.ahrs.getAngle();
        Source<Double> srcAngularErr = Data.source(() -> bot.ahrs.getAngle() - initialAngle);

        bot.snkDrive.bind(Data.source(() -> vec)
                .inter(srcDelta, Data.inter(
                        (out, delta) -> out.y(realSpeed * (delta * deltaFactor + oneMinusDownscale))))
                .inter(srcAngularErr, Data.inter(
                        (out, err) -> out.z(Math.abs(err) <= COR_TURN_THRESH ? 0D : (Math.signum(err) * COR_TURN_MAGN)))));
        bot.snkAutoProfile.bind(srcDelta);
        bot.srcLidarF.get();
        long initialTime = System.currentTimeMillis();
        Flow.waitUntil(timeout > 0L
                ? () -> srcDelta.get() <= errMargin || System.currentTimeMillis() - initialTime > timeout
                : () -> srcDelta.get() <= errMargin);

        Data.popState();
    }

    public static void drive(Robot bot, double inches, double speed) {
        driveWithTimeout(bot, inches, speed, -1L);
    }

    public static void driveTimed(Robot bot, double speed, long time) {
        Data.pushState();

        Vector4 vec = new Vector4(0, -speed, 0, 0);
        bot.snkDrive.bind(Data.source(() -> vec));
        Flow.waitFor(time);

        Data.popState();
    }

    public static void strafeBlind(Robot bot, long ms, double speed) {
        Data.pushState();

        bot.ahrs.reset();
        double initialAngle = bot.ahrs.getAngle();
        Source<Double> srcAngularErr = Data.source(() -> bot.ahrs.getAngle() - initialAngle);

        Vector4 vec = new Vector4(speed, 0, 0, 0);
        bot.snkDrive.bind(Data.source(() -> vec)
                .inter(srcAngularErr, Data.inter(
                        (out, err) -> out.z(Math.abs(err) <= COR_TURN_THRESH ? 0D : (Math.signum(err) * COR_TURN_MAGN)))));
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
                        (out, delta) -> out.z(absSpeed * (delta * deltaFactor + oneMinusDownscale)))));
        bot.snkAutoProfile.bind(srcDelta);
        bot.snkIntake.bind(Data.source(() -> -0.675D));
        Flow.waitUntil(() -> srcDelta.get() <= 0.025D);

        Data.popState();
    }

    public static void skillshot(Robot bot, boolean theQuickness) {
        Data.pushState();
        double speed = theQuickness ? 0.475D : 0.3275D;
        bot.snkIntake.bind(Data.source(() -> speed));
        Flow.waitFor(1500);
        Data.popState();
    }

}
