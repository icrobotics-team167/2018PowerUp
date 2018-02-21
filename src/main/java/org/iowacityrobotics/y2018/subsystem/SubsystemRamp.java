package org.iowacityrobotics.y2018.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.subsystem.StateMachines;
import org.iowacityrobotics.y2018.util.Consts;
import org.iowacityrobotics.y2018.util.Controls;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

public class SubsystemRamp {

    private static final Source<State> state = SourceSystems.CONTROL.button(Consts.CTRL_PRIMARY, Controls.SELECT)
            .map(StateMachines.stateCounter(State.OFF, State.DEPLOYED, State.RAISED));

    public static Source<Double> getServo() {
        return state.map(Data.mapper(state -> state == State.OFF ? 45D : 135D));
    }

    public static Source<DoubleSolenoid.Value> getPiston() {
        return state.map(Data.mapper(state -> state == State.RAISED ? kReverse : kForward));
    }

    private enum State {
        OFF, DEPLOYED, RAISED
    }

}
