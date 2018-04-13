package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.y2018.util.Consts;
import org.iowacityrobotics.y2018.util.Controls;

public class SubsystemLift {

    public static Source<Double> get() {
        return SourceSystems.CONTROL.axis(Consts.CTRL_SECONDARY, Controls.JOY_L_Y)
                .map(MapperSystems.CONTROL.deadZoneD(Consts.JOY_DZ));
    }

}
