package org.iowacityrobotics.y2018.auto.deprecated;

import org.iowacityrobotics.y2018.Robot;
import org.iowacityrobotics.y2018.auto.AutoUtil;
import org.iowacityrobotics.y2018.auto.IAutoRoutine;

public class RoutineScaleSameSwitchOther implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {

        AutoUtil.drive(bot, 324, 0.5);
        AutoUtil.turn(bot, mult * 90, 0.5);
        // TODO Implement

    }

}
