package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineAutoLine implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 2.5, 0.5);
    }

}
