package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

@FunctionalInterface
public interface IAutoRoutine {

    void doTheAutoThing(Robot robot, int mult);

}
