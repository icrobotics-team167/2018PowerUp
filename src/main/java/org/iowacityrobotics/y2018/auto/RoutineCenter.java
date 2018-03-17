package org.iowacityrobotics.y2018.auto;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.y2018.Robot;

public class RoutineCenter implements IAutoRoutine {
    @Override
    public void doTheAutoThing(Robot bot, int mult) {
        AutoUtil.drive(bot, 30D, 0.9D); // drive forwards a bit
//        AutoUtil.strafe(bot, mult * 50D, 1D);
        AutoUtil.strafeBlind(bot, 1000L + 550L * mult, mult * -1D); // strafe towards side with active switch

        Data.pushState();
        bot.snkLift.bind(Data.source(() -> 0.585D)); // while raising the cube...
        AutoUtil.driveWithTimeout(bot, 58D, 0.9D, 2000L); // drive forwards to switch
        Data.popState();

        AutoUtil.skillshot(bot, true); // release the cube
        AutoUtil.driveWithTimeout(bot, 25D, -1D, 750L); // back off so cube holder doesn't get caught

        // TODO Pick up another cube?
//        AutoUtil.turn(bot, mult * 90D, 0.9D);
//
//        Data.pushState();
//        bot.snkIntake.bind(Data.source(() -> -0.675D));
//        AutoUtil.driveWithTimeout(bot, 36D, 0.9D, 2000L);
//        Data.popState();
    }
}
