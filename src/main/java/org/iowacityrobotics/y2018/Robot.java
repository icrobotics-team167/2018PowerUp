package org.iowacityrobotics.y2018;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.Funcs;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Devices;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.IRobotProgram;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.MapperSystems;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.logging.Logs;
import org.iowacityrobotics.roboed.util.math.Maths;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;
import org.iowacityrobotics.roboed.vision.CameraType;
import org.iowacityrobotics.roboed.vision.VisionServer;
import org.iowacityrobotics.y2018.auto.*;
import org.iowacityrobotics.y2018.subsystem.*;
import org.iowacityrobotics.y2018.util.Consts;
import org.iowacityrobotics.y2018.util.Controls;

public class Robot implements IRobotProgram {

    // Gyroscope
    public final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    // Ramp
    public Source<RampState> srcRampState;
    public Source<Double> srcRampWinchA, srcRampWinchB;
    public Sink<Double> snkRampReleaseA, snkRampReleaseB;
    public Sink<Double> snkRampWinchA, snkRampWinchB;

    // Lift
    public Source<Double> srcLift;
    public Source<Double> srcLiftEnc;
    public Sink<Double> snkLift;
    public Sink<Double> snkLiftEnc;
    public LiftController liftController;

    // Drive
    public Sink<Vector4> snkDrive;

    // Intake
    public Source<Double> srcIntake;
    public Sink<Double> snkIntake;

    // LIDAR
    public Source<Double> srcLidarF;
    public Sink<Double> snkLidarF;
    public Source<Double> srcLidarL;
    public Sink<Double> snkLidarL;

    // Auto feedback
    public Sink<Double> snkAutoProfile;

    @Override
    public void init() {
        // Ramp
        srcRampState = new RampStator();
        snkRampReleaseA = SinkSystems.MOTOR.servo(5, RampState.AUTO.a);
        snkRampReleaseB = SinkSystems.MOTOR.servo(6, RampState.AUTO.b);

        Source<Boolean> rampMod = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.X);
        srcRampWinchA = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.START)
                .map(MapperSystems.CONTROL.buttonValue(0D, 1D))
                .inter(rampMod, Data.inter((d, b) -> b ? -d : d));
        srcRampWinchB = SourceSystems.CONTROL.button(Consts.CTRL_SECONDARY, Controls.SELECT)
                .map(MapperSystems.CONTROL.buttonValue(0D, 1D))
                .inter(rampMod, Data.inter((d, b) -> b ? -d : d));
        snkRampWinchA = SinkSystems.MOTOR.victorSp(0);
        snkRampWinchB = SinkSystems.MOTOR.victorSp(1);

        // Lift
        /// BEGIN real lift
        WPI_TalonSRX liftTalon = Devices.talonSrx(5);
        SensorCollection sensors = liftTalon.getSensorCollection();
        Source<Boolean> liftLimit = Data.source(Devices.dioInput(2)::get);
        double[] encOffset = new double[] {sensors.getQuadraturePosition()};
        srcLift = SubsystemLift.get();
        srcLiftEnc = liftLimit.map(Data.mapper(lim -> {
            double pos = sensors.getQuadraturePosition();
            if (!lim) encOffset[0] = pos;
            return Maths.clamp((pos - encOffset[0]) / 27500D, 0D, 1D);
        }));
//        srcLift = srcLift.inter(srcLiftEnc, // (lift feedback controller)
//                Data.inter((v, feedback) -> (feedback >= 0.81D || feedback <= 0.34D)
//                        ? 0.5D * v : v));
        snkLift = SinkSystems.MOTOR.talonSrx(5).join(
                SinkSystems.MOTOR.talonSrx(6)
                        .map(Funcs.invertD()));
        /// END real lift

        /// BEGIN fake lift
        /*FakeLift lift = new FakeLift();
        Source<Boolean> liftLimit = lift.sourceLimit;
        double[] encOffset = new double[] {lift.get()};
        srcLift = SubsystemLift.get();
        srcLiftEnc = liftLimit.map(Data.mapper(lim -> {
            double pos = lift.get();
            if (!lim) encOffset[0] = pos;
            return Maths.clamp((pos - encOffset[0]) / 27500D, 0D, 1D);
        }));
        srcLift = srcLift.inter(srcLiftEnc, // (lift feedback controller)
                Data.inter((v, feedback) -> (feedback >= 0.81D || feedback <= 0.34D)
                        ? 0.5D * v : v));
        snkLift = lift.sink;*/
        /// EMD fake lift

        snkLiftEnc = SinkSystems.DASH.number("Lift Encoder");
        Sink<Boolean> snkLimit = SinkSystems.DASH.string("Lift At Bottom")
                .map(Data.mapper(Object::toString));
        liftController = new LiftController(srcLiftEnc, snkLift);
        SmartDashboard.putNumber("Lift Setpoint", 0D);

        // Drive
        MotorTuple4 motors = MotorTuple4.ofTalons(2, 3, 1, 4);
        snkDrive = SinkSystems.DRIVE.mecanum(motors)
                .map(Data.mapper(v -> v.y(-v.y())));

        // Intake
        srcIntake = SubsystemIntake.get();
        snkIntake = SinkSystems.MOTOR.spark(8).join(
                SinkSystems.MOTOR.spark(9)
                        .map(Funcs.invertD()));

        // LIDAR
        srcLidarF = SourceSystems.SENSOR.lidarLite(0, 39092.0732D);
        snkLidarF = SinkSystems.DASH.number("Front LIDAR");
        srcLidarL = SourceSystems.SENSOR.lidarLite(1, 39092.0732D);
        snkLidarL = SinkSystems.DASH.number("Side LIDAR");

        // Auto feedback
        snkAutoProfile = SinkSystems.DASH.number("Motion profile");

        // Auto control
        SendableChooser<StartPos> startPosCtrl = new SendableChooser<>();
        for (StartPos pos : StartPos.values()) startPosCtrl.addObject(pos.name(), pos);
        startPosCtrl.addDefault(StartPos.LEFT.name(), StartPos.LEFT);
        SmartDashboard.putData("Starting Position", startPosCtrl);

        SendableChooser<AutoGoal> goalCtrl = new SendableChooser<>();
        for (AutoGoal goal : AutoGoal.values()) goalCtrl.addObject(goal.name(), goal);
        goalCtrl.addDefault(AutoGoal.DRIVE_ACROSS_AUTO_LINE.name(), AutoGoal.DRIVE_ACROSS_AUTO_LINE);
        SmartDashboard.putData("Autonomous Goal", goalCtrl);

        SendableChooser<Boolean> twoCtrl = new SendableChooser<>();
        twoCtrl.addDefault("Yes, do two", true);
        twoCtrl.addObject("No, only do one", false);
        SmartDashboard.putData("Do two cubes?", twoCtrl);

        // Camera
        VisionServer.putImageSource("usb cam", VisionServer.getCamera(CameraType.USB, 0));

        // Runmodes
        RobotMode.TELEOP.setOperation(() -> {
            snkLift.bind(srcLift);
//            snkDrive.bind(primaryDriveCtrl.getSelected().source);
            snkDrive.bind(PrimaryDriveScheme.Y_DRIVE.source);
            snkRampReleaseA.bind(srcRampState.map(Data.mapper(s -> s.a)));
            snkRampReleaseB.bind(srcRampState.map(Data.mapper(s -> s.b)));
            snkRampWinchA.bind(srcRampWinchA);
            snkRampWinchB.bind(srcRampWinchB);
            snkIntake.bind(srcIntake);
            snkLidarF.bind(srcLidarF);
            snkLiftEnc.bind(srcLiftEnc);
            snkLimit.bind(liftLimit);
            Flow.waitInfinite();
        });

        RobotMode.AUTO.setOperation(() -> {
            snkRampReleaseA.bind(srcRampState.map(Data.mapper(s -> s.a)));
            snkRampReleaseB.bind(srcRampState.map(Data.mapper(s -> s.b)));
            liftController.bind();
            StartPos startPos = startPosCtrl.getSelected();
            AutoGoal goal = goalCtrl.getSelected();
            IAutoRoutine routine;
            FieldConfig field = getFieldConfiguration();
            boolean switchSame = field.switchSide == startPos, scaleSame = field.scaleSide == startPos;
            switch (goal) {
                case SWITCH_IF_ON_STARTING_POS:
                    if (switchSame) {
                        routine = new RoutineSwitch();
                    } else {
                        routine = new RoutineAutoLine();
                    }
                    break;
                case SWITCH_VIA_STRAFE_IF_ON_STARTING_POS:
                    if (switchSame) {
                        routine = new RoutineSwitchStrafe();
                    } else {
                        routine = new RoutineAutoLineStrafe();
                    }
                    break;
                case FROM_CENTER_DO_SWITCH:
                    startPos = field.switchSide;
                    routine = new RoutineCenter();
                    break;
                case DRIVE_ACROSS_AUTO_LINE:
                    routine = new RoutineAutoLine();
                    break;
                case DO_NOTHING_FOR_30_SECONDS:
                    routine = new RoutineNoop();
                    break;
                default:
                    throw new RuntimeException("wtf how did you do that");
            }
            snkLiftEnc.bind(srcLiftEnc);
            snkLidarF.bind(srcLidarF);
            liftController.setBlocking(0D, this);
            Logs.info("Running strategy {} on side {}",
                    routine.getClass().getSimpleName(),
                    startPos.name());
            routine.doTheAutoThing(this, startPos.mult, twoCtrl.getSelected());
        });

        RobotMode.TEST.setOperation(() -> {
            snkLidarF.bind(srcLidarF);
            snkLiftEnc.bind(srcLiftEnc);
            snkRampReleaseA.bind(srcRampState.map(Data.mapper(s -> s.a)));
            snkRampReleaseB.bind(srcRampState.map(Data.mapper(s -> s.b)));
            liftController.bind();
            liftController.set(0D);
            SmartDashboard.putBoolean("set lift", false);
            SmartDashboard.putBoolean("reset lift", false);
            SmartDashboard.putNumber("Lift Setpoint", 0D);
            Flow.whileWaiting(() -> {
                SmartDashboard.getBoolean("reset lift", false);
                SmartDashboard.getBoolean("set lift", false);
                if (SmartDashboard.getBoolean("reset lift", false)) {
                    liftController.set(0);
                    SmartDashboard.putBoolean("reset lift", false);
                } else if (SmartDashboard.getBoolean("set lift", false)) {
                    liftController.set(0.5D);
                    SmartDashboard.putBoolean("set lift", false);
                }
            });
            Flow.waitInfinite();
        });

        RobotMode.DISABLED.setOperation(() -> {
            liftController.unbind();
        });
    }

    private FieldConfig getFieldConfiguration() {
        DriverStation ds = DriverStation.getInstance();
        String data = ds.getGameSpecificMessage();
        while (data == null && !Thread.currentThread().isInterrupted()) {
            try {
                data = ds.getGameSpecificMessage();
            } catch (Exception ignored) {}
        }
        if (data == null) Flow.end();
        return FieldConfig.parse(data);
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
        SWITCH_IF_ON_STARTING_POS,
        SWITCH_VIA_STRAFE_IF_ON_STARTING_POS,
        FROM_CENTER_DO_SWITCH,
        DRIVE_ACROSS_AUTO_LINE,
        DO_NOTHING_FOR_30_SECONDS
    }

}
