package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;

public class SubsystemLift {

    private static final Source<Double> btnFwd = SourceSystems.CONTROL.button(1, 4)
            .map(MapperSystems.CONTROL.buttonValue(0D, 0.5));

    private static final Source<Double> btnRev = SourceSystems.CONTROL.button(1, 1)
            .map(MapperSystems.CONTROL.buttonValue(0D, -0.5));

    private static final Source<Double> srcCombined = btnFwd.inter(btnRev, Data.inter((a, b) -> a + b));

    public static Source<Double> get() {
        return srcCombined;
    }

}
