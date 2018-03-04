package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineSwitch implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 147.5D, 0.9D); // drive forwards until level with switch
        AutoUtil.turn(bot, mult * -90, 0.75D); // turn towards switch
        bot.liftController.setBlocking(0.15D, bot); // raise cube a bit and wait until it's raised
        AutoUtil.driveWithTimeout(bot, 19.95D, 0.75D, 500L); // drive forwards to switch
        AutoUtil.skillshot(bot, false); // release cube
        AutoUtil.driveWithTimeout(bot, 25D, -1D, 750L); // back off so cube holder doesn't get caught
        // TODO Pick up another cube?
    }

}
