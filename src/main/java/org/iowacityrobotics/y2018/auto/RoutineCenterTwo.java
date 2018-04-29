package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.y2018.Robot;

/**
 * The 12-step suite
 */
public class RoutineCenterTwo implements IAutoRoutine { // TODO update to reflect changes in RoutineCenter
    @Override
    public void doTheAutoThing(Robot bot, int mult, boolean two) {
        RoutineCenter.asdf(bot, mult);

        // fall back and turn towards cube pile
        bot.liftController.set(-0.5D);
        Flow.waitWhile(bot.liftLimit::get);
        bot.liftController.set(0D); // FIXME this won't work with an encoder
        AutoUtil.driveTimed(bot, -0.75D, 800L);
        Logs.info("step 5 done");
        AutoUtil.turn(bot, 45D * mult, 0.9D);
        Logs.info("step 6 done");

        // slowly advance towards cube pile while running intake
        Data.pushState();
        bot.snkIntake.bind(Data.source(() -> -0.675D));
        AutoUtil.driveTimed(bot, 0.35D, 3000L);
        Data.popState();
        Logs.info("step 7 done");

        // back off cube pile and turn back towards switch
        AutoUtil.driveTimed(bot, -0.7D, 1500L);
        Logs.info("step 8 done");
        AutoUtil.turn(bot, -45D * mult, 0.9D);
        Logs.info("step 9 done");

        // approach switch and drop cube
        bot.liftController.set(0.5D);
        AutoUtil.driveTimed(bot, 0.9D, 2055L);
        Flow.waitFor(945L);
        bot.liftController.set(0D); // FIXME this won't work once the encoder is fixed!!!
        Logs.info("step 10 done");
        AutoUtil.skillshot(bot, true);
        Logs.info("step 11 done");

        AutoUtil.driveTimed(bot, -0.9D, 750L); // back off so cube holder doesn't get caught
        bot.liftController.set(0D);
        Logs.info("step 12 done");
    }
 }
