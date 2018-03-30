package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.y2018.Robot;

public class RoutineSwitch implements IAutoRoutine {

    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
        AutoUtil.drive(bot, 137.5D, 0.9D); // drive forwards until level with switch
        AutoUtil.turn(bot, mult * 90, 0.75D); // turn towards switch
        bot.liftController.setBlocking(0.5D, bot); // raise cube a bit and wait until it's raised
        AutoUtil.driveWithTimeout(bot, 19.95D, 0.75D, 500L); // drive forwards to switch
        AutoUtil.skillshot(bot, false); // release cube
        if (two) {
            AutoUtil.strafeBlind(bot, 1000L, mult * -1D); // strafe towards cube row
            AutoUtil.turn(bot, 45D * mult, 0.9D); // turn towards cube
            bot.liftController.setBlocking(0D, bot); // lower lift

            // slowly approach and pick up cube
            Data.pushState();
            bot.snkIntake.bind(Data.source(() -> -0.675D));
            AutoUtil.driveTimed(bot, 0.35D, 2000L);
            Data.popState();

            bot.liftController.setBlocking(0.5D, bot); // raise lift
            AutoUtil.turn(bot, 22.5D * mult, 0.5D); // turn towards switch
            AutoUtil.skillshot(bot, true); // release cube
        }
        AutoUtil.driveTimed(bot, -0.75D, 325L); // back off so cube holder doesn't get caught
    }

}
