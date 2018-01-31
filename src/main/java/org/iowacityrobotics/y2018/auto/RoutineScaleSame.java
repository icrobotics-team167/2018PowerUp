package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineScaleSame implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        // TODO Implement
        AutoUtil.drive(bot, 8.2296, 0.5);
        AutoUtil.turn(bot, mult * 90D, 0.5);

    }

}
