package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.y2018.Robot;

public class RoutineCenter implements IAutoRoutine {
    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 24D, 0.9D); // drive forwards a bit
        Logs.info("step 1 done");
        AutoUtil.strafeBlind(bot, 1350L + 335L * mult, mult * -1D); // strafe towards side with active switch
        Logs.info("step 2 done");

//        Data.pushState();
//        bot.snkLift.bind(Data.source(() -> -0.480D)); // while raising the cube...
        bot.liftController.set(0.5D);
        AutoUtil.driveWithTimeout(bot, 79D, 0.9D, 2000L); // drive forwards to switch
//        Data.popState();
        Logs.info("step 3 done");

        AutoUtil.skillshot(bot, true); // release the cube
        AutoUtil.driveWithTimeout(bot, 25D, -1D, 750L); // back off so cube holder doesn't get caught
        Logs.info("step 4 done");
        bot.liftController.set(0D);

        // TODO Pick up another cube?
//        AutoUtil.turn(bot, mult * 90D, 0.9D);
//
//        Data.pushState();
//        bot.snkIntake.bind(Data.source(() -> -0.675D));
//        AutoUtil.driveWithTimeout(bot, 36D, 0.9D, 2000L);
//        Data.popState();
    }
}
