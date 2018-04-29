package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.y2018.Robot;

/**
 * The 12-step suite
 */
public class RoutineCenterTwoSquare implements IAutoRoutine { // TODO update to reflect changes in RoutineCenter
    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
        if (!two) {
            new RoutineCenter().doTheAutoThing(bot, mult, false);
            return;
        }
        double dist = RoutineCenter.asdf(bot, mult);

        bot.liftController.set(-0.36D); // inverse of step 4
        AutoUtil.driveTimedSpline(bot, -dist, 0.9D, 3000L);
        bot.liftController.set(0D); // FIXME this won't work once the encoder is fixed!!!
        Logs.info("step 5 done");

        AutoUtil.strafeBlind(bot, 1350L + 335L * mult, mult); // inverse of step 3
        Logs.info("step 6 done");

        Data.pushState();
        double a = bot.srcLidarF.get();
        bot.snkIntake.bind(Data.source(() -> -0.675D)); // intake cube
        AutoUtil.driveTimedSpline(bot, dist * 0.5D, 0.9D, 1500L); // pick up a cube
        a = Math.abs(bot.srcLidarF.get() - a);
        Data.popState();
        Logs.info("step 7 done");

        AutoUtil.driveTimedSpline(bot, -a, 0.9D, 1500L); // inverse of step 7
        Logs.info("step 8 done");

        // just do the whole thing again
        AutoUtil.strafeBlind(bot, 1350L + 335L * mult, mult * -1D); // strafe towards side with active switch
        Logs.info("step 9 (2) done");

        bot.liftController.set(0.36D); // instruct the lift to hold the cube in the air
        AutoUtil.driveTimedSpline(bot, 112D, 0.9D, 3000L); // drive forwards to switch
        bot.liftController.set(0D); // FIXME this won't work once the encoder is fixed!!!
        Logs.info("step 10 (3) done");

        AutoUtil.skillshot(bot, true); // release the cube
        Logs.info("step 11 (4) done");

        AutoUtil.driveWithTimeout(bot, -25D, 1D, 750L); // back off so cube holder doesn't get caught
        bot.liftController.set(0D);
        Logs.info("step 12 done");
    }
 }
