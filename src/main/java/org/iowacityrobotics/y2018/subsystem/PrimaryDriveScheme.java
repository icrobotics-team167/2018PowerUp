package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.subsystem.StateMachines;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.y2018.util.Consts;
import org.iowacityrobotics.y2018.util.Controls;
import org.iowacityrobotics.y2018.util.SpeedLevel;

import java.util.function.Supplier;

public enum PrimaryDriveScheme {
    
    R_TRIGGER_DRIVE(() -> {
        Source<Boolean> srcReverse = SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.ZR)
                .map(MapperSystems.CONTROL.toggle());
        return SourceSystems.CONTROL.dualJoy(Consts.CTRL_PRIMARY)
                .map(MapperSystems.CONTROL.deadZone2V2(Consts.JOY_DZ))
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .inter(SourceSystems.CONTROL.axis(Consts.CTRL_PRIMARY, Controls.R_AXIS),
                        Data.inter((v, t) -> v.multiply2D(0.5D + 0.5D * t)))
                .inter(srcReverse, Data.inter((v, r) -> r ? v.multiply2D(-1D) : v));
    }),
    L_TRIGGER_DRIVE(() -> {
        Source<Boolean> srcReverse = SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.ZL)
                .map(MapperSystems.CONTROL.toggle());
        return SourceSystems.CONTROL.dualJoy(Consts.CTRL_PRIMARY)
                .map(MapperSystems.CONTROL.deadZone2V2(Consts.JOY_DZ))
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .inter(SourceSystems.CONTROL.axis(Consts.CTRL_SECONDARY, Controls.L_AXIS),
                        Data.inter((v, t) -> v.multiply2D(0.5D + 0.5D * t)))
                .inter(srcReverse, Data.inter((v, r) -> r ? v.multiply2D(-1D) : v));
    }),
    Y_DRIVE(() -> {
        Source<SpeedLevel> srcSpeed = SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.A)
                .inter(SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.B),
                        Data.inter((a, b) -> a ? SpeedLevel.FAST : b ? SpeedLevel.FASTER : SpeedLevel.NIL))
                .inter(SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.Y),
                        Data.inter((a, b) -> a.invalid && b ? SpeedLevel.FASTEST : a))
                .map(StateMachines.stateLatch(SpeedLevel.NIL, SpeedLevel.FASTER));
        Source<Double> srcSpeedAugment = SourceSystems.CONTROL.axis(Consts.CTRL_PRIMARY, Controls.L_AXIS);
        Source<Boolean> srcReverse = SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.START)
                .map(MapperSystems.CONTROL.toggle());
        return SourceSystems.CONTROL.dualJoy(Consts.CTRL_PRIMARY)
                .map(MapperSystems.CONTROL.deadZone2V2(Consts.JOY_DZ))
                .map(MapperSystems.DRIVE.dualJoyMecanum())
                .inter(srcSpeed, Data.inter((v, t) -> v.multiply2D(t.factor)))
                .inter(srcSpeedAugment, Data.inter((v, t) -> v.multiply2D(1D - 0.6D * t)))
                .inter(srcReverse, Data.inter((v, r) -> r ? v.multiply2D(-1D) : v));
    });
    
    public final Source<Vector4> source;
    
    PrimaryDriveScheme(Supplier<Source<Vector4>> factory) {
        this.source = factory.get();
    }
    
}
