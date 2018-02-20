package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.y2018.Robot;

public class RoutineCenterToRight implements IAutoRoutine {
    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        Logs.info("Going right!");
        AutoUtil.drive(bot, 40D, 0.9D);
        AutoUtil.strafe(bot, 46D, 0.9D);
        AutoUtil.drive(bot, 50D, 0.9D);
        AutoUtil.skillshot(bot, false);
    }
}
