package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineSwitch implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 168, 0.5);
        AutoUtil.turn(bot, mult * 90, 0.5);
        // TODO Implement
    }

}
