package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineScaleOther implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 221.5D, 0.9D); // drive forwards to midpoint of platform and cube line
        AutoUtil.turn(bot, mult * 90, 0.75D); // turn towards opposite side of field
        AutoUtil.drive(bot, 232.9D, 0.9D); // cross field to opposite starting point's line
        AutoUtil.turn(bot, mult * -90, 0.75D); // turn back forwards
        AutoUtil.drive(bot, 82.06D, 0.9D); // drive forwards until level with scale
        AutoUtil.turn(bot, mult * -90, 0.75D); // turn towards scale
        RoutineScaleSame.doScalePlacement(bot); // place the cube
        // TODO Pick up another cube?
    }

}
