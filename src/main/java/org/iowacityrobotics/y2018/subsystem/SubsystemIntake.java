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

    private static final Source<Double> srcOutSlow = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.B)
            .map(MapperSystems.CONTROL.buttonValue(0D, 0.575D));

    private static final Source<Double> srcOut = SourceSystems.CONTROL.axis(Consts.CTRL_SECONDARY, Controls.L_AXIS);

    private static final Source<Double> srcOutPrimCtrl = SourceSystems.CONTROL.axis(Consts.CTRL_PRIMARY, Controls.R_AXIS);

    private static final Source<Double> srcInSlow = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.A)
            .map(MapperSystems.CONTROL.buttonValue(0D, -0.575D));

    private static final Source<Double> srcIn = SourceSystems.CONTROL.axis(Consts.CTRL_SECONDARY, Controls.R_AXIS)
            .map(Funcs.invertD());

    private static final Source<Double> srcCombined = srcOutSlow
            .inter(srcOut, Funcs.sumD())
            .inter(srcInSlow, Funcs.sumD())
            .inter(srcIn, Funcs.sumD())
            .inter(srcOutPrimCtrl, Funcs.sumD())
            .map(Data.mapper(v -> Maths.clamp(v, -1D, 1D)));

    public static Source<Double> get() {
        return srcCombined;
    }

}
