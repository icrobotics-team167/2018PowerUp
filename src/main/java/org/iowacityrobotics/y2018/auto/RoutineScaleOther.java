package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineScaleOther implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        // TODO Implement
        AutoUtil.drive(bot, 220, 1);
        AutoUtil.turn(bot, 90 * mult, .75);
        AutoUtil.drive(bot, 75, 1);
        AutoUtil.turn(bot, -90 * mult, .75);
        AutoUtil.drive(bot, 3, .75);
        AutoUtil.turn(bot,40 * mult, 1);
        
    }

}
