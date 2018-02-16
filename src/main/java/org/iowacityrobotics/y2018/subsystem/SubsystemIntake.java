package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.Funcs;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.math.Maths;
import org.iowacityrobotics.y2018.util.Consts;
import org.iowacityrobotics.y2018.util.Controls;

public class SubsystemIntake {

    private static final Source<Double> srcOut = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.B)
            .map(MapperSystems.CONTROL.buttonValue(0D, 0.675D));

    private static final Source<Double> srcOutFast = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.Y)
            .map(MapperSystems.CONTROL.buttonValue(0D, 0.82D));

    private static final Source<Double> srcIn = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.A)
            .map(MapperSystems.CONTROL.buttonValue(0D, -1D));

    private static final Source<Double> srcCombined = srcOut
            .inter(srcOutFast, Funcs.sumD())
            .inter(srcIn, Funcs.sumD())
            .map(Data.mapper(v -> Maths.clamp(v, -1D, 1D)));

    public static Source<Double> get() {
        return srcCombined;
    }

}
