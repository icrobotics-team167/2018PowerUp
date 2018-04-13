package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.y2018.Robot;

public class RoutineCenter implements IAutoRoutine {
    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
        if (two) {
            new RoutineCenterTwo().doTheAutoThing(bot, mult, true);
            return;
        }
        AutoUtil.drive(bot, 12D, 0.9D); // drive forwards a bit
        Logs.info("step 1 done");
        AutoUtil.strafeBlind(bot, 1350L + 335L * mult, mult * -1D); // strafe towards side with active switch
        Logs.info("step 2 done");

        bot.liftController.set(0.36D); // instruct the lift to hold the cube in the air
        AutoUtil.driveTimedSpline(bot, 112D, 0.9D, 3000L); // drive forwards to switch
        bot.liftController.set(0D); // FIXME this won't work once the encoder is fixed!!!
        Logs.info("step 3 done");

        AutoUtil.skillshot(bot, true); // release the cube
        Logs.info("step 4 done");

        AutoUtil.driveWithTimeout(bot, -25D, 1D, 750L); // back off so cube holder doesn't get caught
        bot.liftController.set(0D);
    }
 }
