package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.y2018.Robot;

public class RoutineCenter implements IAutoRoutine {
    static double asdf(Robot bot, int mult) {
        AutoUtil.driveTimed(bot, 0.5D,800L); // drive forwards a bit
        Logs.info("step 1 done");
        AutoUtil.strafeBlind(bot, 1350L + 335L * mult, mult * -1D); // strafe towards side with active switch
        Logs.info("step 2 done");

        bot.liftController.set(0.36D); // instruct the lift to hold the cube in the air
        double a = bot.srcLidarF.get();
        AutoUtil.driveTimed(bot, 0.5D,2000L); // drive forwards to switch
        a = Math.abs(bot.srcLidarF.get() - a);
        bot.liftController.set(0D); // FIXME this won't work once the encoder is fixed!!!
        Logs.info("step 3 done");

        AutoUtil.skillshot(bot, true); // release the cube
        Logs.info("step 4 done");
        return a;
    }

    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
        if (two) {
            new RoutineCenterTwo().doTheAutoThing(bot, mult, true);
            return;
        }
        asdf(bot, mult);

        AutoUtil.driveWithTimeout(bot, -25D, 1D, 750L); // back off so cube holder doesn't get caught
        bot.liftController.set(0D);
        Logs.info("step 5 done");
    }
 }
