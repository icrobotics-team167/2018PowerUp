package org.iowacityrobotics.y2018;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.IRobotProgram;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;
import org.iowacityrobotics.y2018.auto.*;
import org.iowacityrobotics.y2018.subsystem.SubsystemDrive;
import org.iowacityrobotics.y2018.subsystem.SubsystemIntake;
import org.iowacityrobotics.y2018.subsystem.SubsystemLift;
import org.iowacityrobotics.y2018.subsystem.SubsystemRamp;

import java.util.function.Supplier;

public class Robot implements IRobotProgram {

    // Ramp
    public Source<Double> srcRampServo;
    public Sink<Double> snkRampServo;
    public Source<DoubleSolenoid.Value> srcRampPiston;
    public Sink<DoubleSolenoid.Value> snkRampPiston;

    // Lift
    public Source<Double> srcLift;
    public Sink<Double> snkLift;

    // Drive
    public Source<Vector4> srcDrive;
    public Sink<Vector4> snkDrive;

    // Intake
    public Source<Double> srcIntakeFwd;
    public Source<Double> srcIntakeRev;
    public Sink<Double> snkIntakeFwd;
    public Sink<Double> snkIntakeRev;

    // Stupid auto routine trees
    private static final Branch<Branch<Supplier<IAutoRoutine>>> autoTreeSwitchScale = new Branch<>(
            new Branch<>(
                    RoutineSwitchSameScaleSame::new,
                    RoutineSwitchSameScaleOther::new
            ),
            new Branch<>(
                    RoutineSwitchOtherScaleSame::new,
                    RoutineSwitchOtherScaleOther::new
            )
    );
    private static final Branch<Branch<Supplier<IAutoRoutine>>> autoTreeScaleSwitch = new Branch<>(
            new Branch<>(
                    RoutineScaleSameSwitchSame::new,
                    RoutineScaleSameSwitchOther::new
            ),
            new Branch<>(
                    RoutineScaleOtherSwitchSame::new,
                    RoutineScaleOtherSwitchOther::new
            )
    );

    @Override
    public void init() {
        // Ramp
        srcRampPiston = SubsystemRamp.getPiston();
        srcRampServo = SubsystemRamp.getServo();
        snkRampServo = SinkSystems.MOTOR.servo(0);
        snkRampPiston = SinkSystems.OTHER.dblSolenoid(0, 1);

        // Lift
        srcLift = SubsystemLift.get();
        snkLift = SinkSystems.MOTOR.talonSrx(2);

        // Drive
        srcDrive = SubsystemDrive.get();
        MotorTuple4 motors = MotorTuple4.ofTalons(1, 4, 3, 6);
        motors.getFrontRight().setInverted(true);
        motors.getRearRight().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(motors);

        // Intake
        srcIntakeFwd = SubsystemIntake.getForwards();
        srcIntakeRev = SubsystemIntake.getReverse();
        snkIntakeFwd = SinkSystems.MOTOR.talonSrx(5);
        snkIntakeRev = SinkSystems.MOTOR.talonSrx(7);

        // Auto control
        SendableChooser<StartPos> startPosCtrl = new SendableChooser<>();
        for (StartPos pos : StartPos.values()) startPosCtrl.addObject(pos.name(), pos);
        SmartDashboard.putData("Starting Position", startPosCtrl);

        SendableChooser<AutoGoal> goalCtrl = new SendableChooser<>();
        for (AutoGoal goal : AutoGoal.values()) goalCtrl.addObject(goal.name(), goal);
        SmartDashboard.putData("Autonomous Goal", goalCtrl);

        // Runmodes
        RobotMode.TELEOP.setOperation(() -> {
            snkRampServo.bind(srcRampServo);
            snkRampPiston.bind(srcRampPiston);
            snkLift.bind(srcLift);
            snkDrive.bind(srcDrive);
            snkIntakeFwd.bind(srcIntakeFwd);
            snkIntakeRev.bind(srcIntakeRev);
            Flow.waitInfinite();
        });

        RobotMode.AUTO.setOperation(() -> {
            StartPos startPos = startPosCtrl.getSelected();
            AutoGoal goal = goalCtrl.getSelected();
            IAutoRoutine routine;
            if (startPos == StartPos.CENTER || goal == AutoGoal.CROSS_AUTO_LINE) {
                routine = new RoutineAutoLine(); // Trivial case; we only do the "break auto line" routine here
            } else {
                FieldConfig field = getFieldConfiguration();
                boolean switchSame = field.switchSide == startPos, scaleSame = field.scaleSide == startPos;
                switch (goal) {
                    case SWITCH:
                        routine = switchSame ? new RoutineSwitch() : new RoutineAutoLine();
                        break;
                    case SCALE:
                        routine = switchSame ? new RoutineScaleSame() : new RoutineScaleOther();
                        break;
                    case SWITCH_THEN_SCALE:
                        routine = autoTreeSwitchScale.get(switchSame).get(scaleSame).get();
                        break;
                    case SCALE_THEN_SWITCH:
                        routine = autoTreeScaleSwitch.get(scaleSame).get(switchSame).get();
                        break;
                    default:
                        throw new RuntimeException("wtf how did you do that");
                }
            }
            routine.doTheAutoThing(this, startPos.mult);
        });
    }

    private FieldConfig getFieldConfiguration() {
        return FieldConfig.parse(DriverStation.getInstance().getGameSpecificMessage());
    }

    private enum FieldConfig {
        RR(StartPos.RIGHT, StartPos.RIGHT),
        RL(StartPos.RIGHT, StartPos.LEFT),
        LR(StartPos.LEFT, StartPos.RIGHT),
        LL(StartPos.LEFT, StartPos.LEFT);

        public final StartPos switchSide, scaleSide;

        FieldConfig(StartPos switchSide, StartPos scaleSide) {
            this.switchSide = switchSide;
            this.scaleSide = scaleSide;
        }

        static FieldConfig parse(String cfg) {
            return FieldConfig.valueOf(cfg.substring(0, 2));
        }
    }

    private enum StartPos {
        LEFT(1),
        CENTER(0),
        RIGHT(-1);

        public final int mult;

        StartPos(int mult) {
            this.mult = mult;
        }
    }

    private enum AutoGoal {
        SWITCH,
        SCALE,
        SWITCH_THEN_SCALE,
        SCALE_THEN_SWITCH,
        CROSS_AUTO_LINE
    }

}
