package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineAutoLine implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
//        AutoUtil.drive(bot, 100D, 0.9D);
        AutoUtil.driveTimed(bot, 0.64D, 2000L);
    }

}
