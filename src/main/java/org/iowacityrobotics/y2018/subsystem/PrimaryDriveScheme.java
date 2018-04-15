package org.iowacityrobotics.y2018.subsystem;

import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.math.Maths;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.y2018.Robot;
import org.iowacityrobotics.y2018.auto.AutoUtil;
import org.iowacityrobotics.y2018.util.Consts;
import org.iowacityrobotics.y2018.util.Controls;
import org.iowacityrobotics.y2018.util.SpeedLevel;

import java.util.function.Function;

public enum PrimaryDriveScheme {

    Y_DRIVE(bot -> {
//        Source<SpeedLevel> srcSpeed = SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.A)
//                .inter(SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.B),
//                        Data.inter((a, b) -> a ? SpeedLevel.FAST : b ? SpeedLevel.FASTER : SpeedLevel.NIL))
//                .inter(SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.Y),
//                        Data.inter((a, b) -> a.invalid && b ? SpeedLevel.FASTEST : a))
//                .map(StateMachines.stateLatch(SpeedLevel.NIL, SpeedLevel.FASTER));
        Source<SpeedLevel> srcSpeed = Data.source(() -> SpeedLevel.FASTEST);
        Source<Double> srcSpeedAugment = SourceSystems.CONTROL.axis(Consts.CTRL_PRIMARY, Controls.R_AXIS);
        Source<Boolean> srcReverse = SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.A)
                .map(MapperSystems.CONTROL.toggle());
        Source<Vector4> srcDrive = SourceSystems.CONTROL.dualJoy(Consts.CTRL_PRIMARY)
                .map(MapperSystems.CONTROL.deadZone2V2(Consts.JOY_DZ))
                .map(MapperSystems.DRIVE.dualJoyMecanum());
        double[] fixedAngle = new double[] {bot.ahrs.getAngle()};
        long[] lastAngleChange = new long[] {System.currentTimeMillis()};
        Source<Double> srcLastFixedAngle = srcDrive
                .map(Data.mapper(v -> {
                    long time = System.currentTimeMillis();
                    if (Math.abs(v.z()) > 0.001D) {
                        fixedAngle[0] = bot.ahrs.getAngle();
                        lastAngleChange[0] = time;
                    } else if (time - lastAngleChange[0] < 100L) {
                        fixedAngle[0] = bot.ahrs.getAngle();
                    }
                    return fixedAngle[0];
                }));
        Source<Double> srcAngularErr = srcLastFixedAngle.map(
                Data.mapper(angle -> bot.ahrs.getAngle() - angle));
        return srcDrive
                .inter(srcSpeed, Data.inter((v, t) -> v.multiply2D(t.factor)))
                .inter(srcSpeedAugment, Data.inter((v, t) -> v.multiply2D(1 - 0.4D * t)))
                .inter(srcReverse, Data.inter((v, r) -> r ? v.multiply2D(-1D) : v))
                .inter(srcAngularErr, Data.inter(
                        (out, err) -> out.z(Maths.clamp(out.z() + (Math.abs(err) <= AutoUtil.COR_TURN_THRESH
                                ? 0 : (Math.signum(err) * AutoUtil.COR_TURN_MAGN)), -1D, 1D))));
    });
    
    public final Function<Robot, Source<Vector4>> source;
    
    PrimaryDriveScheme(Function<Robot, Source<Vector4>> factory) {
        this.source = factory;
    }
    
}
