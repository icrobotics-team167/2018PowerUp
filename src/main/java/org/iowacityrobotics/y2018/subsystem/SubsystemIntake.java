package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.math.Maths;

public class SubsystemIntake {

    private static final Source<Double> srcOut = SourceSystems.CONTROL.axis(2, 2)
            .map(MapperSystems.CONTROL.deadZoneD(0.05D));

    private static final Source<Double> srcIn = SourceSystems.CONTROL.axis(2, 3)
            .map(MapperSystems.CONTROL.booleanify(0.5D))
            .map(MapperSystems.CONTROL.buttonValue(0D, -1D));

    private static final Source<Double> srcCombined = srcOut
            .inter(srcIn, Data.inter((a, b) -> a + b))
            .map(Data.mapper(v -> Maths.clamp(v, -1D, 1D)));

    public static Source<Double> getForwards() {
        return srcCombined;
    }

    public static Source<Double> getReverse() {
        return srcCombined.map(Data.mapper(a -> -a));
    }

}
