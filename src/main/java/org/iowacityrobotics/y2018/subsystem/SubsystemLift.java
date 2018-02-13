package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;

public class SubsystemLift {

    public static Source<Double> get() {
        return SourceSystems.CONTROL.axis(2, 5)
                .map(Data.mapper(v -> v * 0.5D));
    }

}
