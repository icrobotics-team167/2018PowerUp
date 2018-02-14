package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineScaleOtherSwitchOther implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        // TODO Implement
        AutoUtil.drive(bot,210, 1);
    }

}
