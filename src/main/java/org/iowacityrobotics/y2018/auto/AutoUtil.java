package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.y2018.Robot;

public class AutoUtil {

    private static final double MIN_MOTOR_MAGN = 0.35D;
    private static final double MIN_TURN_MAGN = 0.50D;

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

    public static void nowBeginsThePerformance(Robot bot) {
        Data.pushState();
        bot.snkIntake.bind(Data.source(() -> -0.675D));

        Data.pushState();
        Vector4 vec = new Vector4(0, -1, 0, 0);
        bot.snkDrive.bind(Data.source(() -> vec));
        Flow.waitFor(300);
        Data.popState();

        Flow.waitFor(150);

        Data.pushState();
        vec.y(1);
        bot.snkDrive.bind(Data.source(() -> vec));
        Flow.waitFor(300);
        Data.popState();

        Data.popState();
    }

    public static void skillshot(Robot bot, boolean theQuickness) {
        Data.pushState();
        double speed = theQuickness ? 0.875D : 0.675D;
        bot.snkIntake.bind(Data.source(() -> speed));
        Flow.waitFor(750);
        Data.popState();
    }

//    public static void driveTime(Robot bot, double seconds, double speed) {
//        Data.pushState();
//
//        bot.ahrs.reset();
//        Vector4 vec = new Vector4(0, speed, 0, 0);
//        MotorTuple4 motors = MotorTuple4.ofTalons(1, 4, 3, 6);
//        motors.getFrontRight().setInverted(true);
//        motors.getRearRight().setInverted(true);
//
//        Sink<Vector4> snkDrive = SinkSystems.DRIVE.mecanum(motors);
//        RobotMode.AUTO.setOperation(() -> {
//            Vector4 vec2 = new Vector4(0, 0.45, 0, 0);
//            Source<Vector4> src = Data.source(() -> vec2);
//            snkDrive.bind(src);
//            Flow.waitFor((long)seconds, TimeUnit.SECONDS);
//        });
//
//        Data.popState();
//    }
}
