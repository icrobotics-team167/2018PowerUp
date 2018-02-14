package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;
import org.iowacityrobotics.y2018.Robot;

import java.util.concurrent.TimeUnit;

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

    public static void driveTime(Robot bot, double seconds, double speed) {
        Data.pushState();

        bot.ahrs.reset();
        Vector4 vec = new Vector4(0, speed, 0, 0);
        MotorTuple4 motors = MotorTuple4.ofTalons(1, 4, 3, 6);
        motors.getFrontRight().setInverted(true);
        motors.getRearRight().setInverted(true);

        Sink<Vector4> snkDrive = SinkSystems.DRIVE.mecanum(motors);
        RobotMode.AUTO.setOperation(() -> {
            Vector4 vec2 = new Vector4(0, 0.45, 0, 0);
            Source<Vector4> src = Data.source(() -> vec2);
            snkDrive.bind(src);
            Flow.waitFor((long)seconds, TimeUnit.SECONDS);
        });

        Data.popState();
    }
}
