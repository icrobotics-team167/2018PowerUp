package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineAutoLineStrafe implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
        AutoUtil.strafeBlind(bot, RoutineSwitchStrafe.ASDF, -mult); // strafe until level with switch
    }

}
