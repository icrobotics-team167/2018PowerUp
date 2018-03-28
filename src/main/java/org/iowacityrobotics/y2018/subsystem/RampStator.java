package org.iowacityrobotics.y2018.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.y2018.util.Consts;
import org.iowacityrobotics.y2018.util.Controls;

public class RampStator extends Source<RampState> {

    private boolean active;

    public RampStator() {
        this.active = false;
    }

    @Override
    public RampState get() {
        switch (RobotMode.get()) {
            case AUTO:
                return RampState.AUTO;
            case TELEOP:
                if (active) return RampState.DOWN;
                if (shouldFire()) {
                    active = true;
                    return RampState.DOWN;
                }
            default:
                return RampState.UP;
        }
    }

    private static boolean shouldFire() {
        DriverStation ds = DriverStation.getInstance();
        return ds.getStickButton(Consts.CTRL_PRIMARY, Controls.ZL)
                && ds.getStickButton(Consts.CTRL_PRIMARY, Controls.ZR)
                && ds.getStickButton(Consts.CTRL_PRIMARY, Controls.START);
    }

    @Override
    public void reset(boolean temp) {
        if (!temp) active = false;
    }

}
