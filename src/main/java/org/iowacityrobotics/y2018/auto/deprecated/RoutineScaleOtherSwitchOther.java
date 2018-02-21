package org.iowacityrobotics.y2018.auto.deprecated;

import org.iowacityrobotics.y2018.Robot;
import org.iowacityrobotics.y2018.auto.AutoUtil;
import org.iowacityrobotics.y2018.auto.IAutoRoutine;

public class RoutineScaleOtherSwitchOther implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        // TODO Implement
        AutoUtil.drive(bot,210, 1);
    }

}
