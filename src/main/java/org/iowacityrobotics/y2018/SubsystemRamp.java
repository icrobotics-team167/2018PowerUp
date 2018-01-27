package org.iowacityrobotics.y2018;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;

public class SubsystemRamp {

    private static final Source<Double> btn = SourceSystems.CONTROL.button(1, 3)
            .map(MapperSystems.CONTROL.buttonValue(0D, 0.5));

    public static Source<Double> getServo() {
        // TODO Implement
    }

    public static Source<DoubleSolenoid.Value> getPiston() {
        // TODO Implement
    }

}
