package org.iowacityrobotics.y2018;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;

public class SubsystemIntake {

    private static final Source<Double> btnFwd = SourceSystems.CONTROL.button(1, 6)
            .map(MapperSystems.CONTROL.buttonValue(0D, 0.5));

    private static final Source<Double> btnRev = SourceSystems.CONTROL.button(1, 5)
            .map(MapperSystems.CONTROL.buttonValue(0D, -0.5));

    private static final Source<Double> srcCombined = btnFwd.inter(btnRev, Data.inter((a, b) -> a + b));

    public static Source<Double> getForwards() {
        return srcCombined;
    }

    public static Source<Double> getReverse() {
        return srcCombined.map(Data.mapper(a -> a * -1));
    }

}
