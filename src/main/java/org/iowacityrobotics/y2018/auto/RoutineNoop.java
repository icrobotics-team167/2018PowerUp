package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineNoop implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot robot, int mult, boolean two) {
        // NO-OP
    }

}
