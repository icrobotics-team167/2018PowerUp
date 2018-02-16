package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;

public class SubsystemLift {

    public static Source<Double> get() {
        return SourceSystems.CONTROL.axis(2, 1)
                .map(MapperSystems.CONTROL.deadZoneD(0.08D));
        
    }

}
