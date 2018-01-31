package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.function.ICondition;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.y2018.Robot;

public class AutoUtil {

    public static void drive(Robot bot, double inches, double speed) {
        Data.pushState();

        Vector4 vec = new Vector4(0, speed, 0, 0);
        bot.snkDrive.bind(Data.source(() -> vec));
        Flow.waitUntil((ICondition)null); // TODO implement this later

        Data.popState();
    }

    public static void turn(Robot bot, double degrees, double speed) {
        Data.pushState();

        speed = Math.abs(speed) * Math.signum(degrees);
        Vector4 vec = new Vector4(0, 0, speed, 0);
        double initialAngle = bot.ahrs.getAngle();
        bot.snkDrive.bind(Data.source(() -> vec));
        Flow.waitUntil(() -> Math.abs(bot.ahrs.getAngle() - initialAngle) >= degrees); // TODO implement this better

        Data.popState();
    }

}
