package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineScaleSameSwitchSame implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {

        AutoUtil.drive(bot, 8.2296, 0.5);
        AutoUtil.turn(bot, mult * 90, 0.5);

        AutoUtil.turn(bot, mult * 90, 0.5);
        AutoUtil.drive(bot, 3.9624, 0.5);
        AutoUtil.turn(bot, mult * 90 * -1, 0.5);
        // TODO Implement
    }

}
