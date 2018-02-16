package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineSwitchSameScaleOther implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 168, 1);
        AutoUtil.turn(bot, mult * 90, 1);

        // TODO Implement block placement in switch

        AutoUtil.turn(bot, mult * -90, .75);
        AutoUtil.drive(bot, 70, 1);
        AutoUtil.turn(bot, mult * 90, .75);
        AutoUtil.drive(bot, 100, 1);
        AutoUtil.turn(bot, -90 * mult, .75);
        AutoUtil.driveTime(bot, 2, 1);
        AutoUtil.drive(bot, 40 * mult, .75);
    }

}
