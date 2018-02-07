package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;

public class SubsystemUltrasonic {

    public static Source<Double> get() {
        return SourceSystems.SENSOR.analog(0)
                .map(Data.mapper(a -> (double) a));
    }

}
