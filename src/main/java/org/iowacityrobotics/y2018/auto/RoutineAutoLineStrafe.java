package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineAutoLineStrafe implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
        AutoUtil.strafeFeedback(bot, -100D * mult, 1D);
    }

}
