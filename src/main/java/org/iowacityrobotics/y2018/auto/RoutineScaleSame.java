package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.y2018.Robot;

public class RoutineScaleSame implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 309.56D, 0.9D); // drive forwards until level with scale
        AutoUtil.turn(bot, mult * 90, 0.75D); // turn towards scale
        doScalePlacement(bot); // place the cube
        // TODO Pick up another cube?
    }

    static void doScalePlacement(Robot bot) {
        bot.liftController.setBlocking(1D, bot); // raise cube to top and wait until it's raised
        AutoUtil.driveWithTimeout(bot, 6.01D, 0.75D, 175L); // drive forwards to scale
        AutoUtil.skillshot(bot, false); // release cube
        AutoUtil.driveTimed(bot, -0.75D, 175L); // back off so cube holder doesn't get caught
    }

}
