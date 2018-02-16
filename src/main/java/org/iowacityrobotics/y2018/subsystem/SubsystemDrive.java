package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.subsystem.StateMachines;
import org.iowacityrobotics.roboed.util.math.Vector4;

public class SubsystemDrive {

    public static Source<Vector4> get() {
        Source<SpeedLevel> srcSpeed = SourceSystems.CONTROL.button(1, 1)
                .inter(SourceSystems.CONTROL.button(1, 2),
                        Data.inter((a, b) -> a ? SpeedLevel.FAST : b ? SpeedLevel.FASTER : SpeedLevel.NIL))
                .inter(SourceSystems.CONTROL.button(1, 4),
                        Data.inter((a, b) -> a.invalid && b ? SpeedLevel.FASTEST : a))
                .map(StateMachines.stateLatch(SpeedLevel.NIL, SpeedLevel.FASTER));
        Source<Boolean> srcReverse = SourceSystems.CONTROL.button(1, 8)
                .map(MapperSystems.CONTROL.toggle());
        return SourceSystems.CONTROL.dualJoy(1)
                .map(MapperSystems.CONTROL.deadZone2V2(0.08D))
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .inter(srcSpeed, Data.inter((v, t) -> v.multiply2D(t.factor)))
                //.inter(SourceSystems.CONTROL.axis(1, 3), Data.inter((v, t) -> v.multiply2D(0.75D + 0.25D * t)))
                .inter(srcReverse, Data.inter((v, r) -> r ? v.multiply2D(-1D) : v));
    }

    private enum SpeedLevel {
        NIL(),
        FAST(1D / 3D),
        FASTER(2D / 3D),
        FASTEST(1D);

        public double factor;
        public boolean invalid;

        SpeedLevel(double factor) {
            this.factor = factor;
            this.invalid = false;
        }

        SpeedLevel() {
            this.factor = 0D;
            this.invalid = true;
        }
    }

}
