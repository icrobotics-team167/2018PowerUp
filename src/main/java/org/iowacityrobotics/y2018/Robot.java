package org.iowacityrobotics.y2018;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.subsystem.SourceSystems;
import org.iowacityrobotics.roboed.util.math.Maths;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;
import org.iowacityrobotics.roboed.vision.CameraType;
import org.iowacityrobotics.roboed.vision.VisionServer;
import org.iowacityrobotics.y2018.auto.*;
import org.iowacityrobotics.y2018.subsystem.*;

public class Robot implements IRobotProgram {

    // Gyroscope
    public final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    // Ramp
    public Source<Double> srcRampServo;
    public Sink<Double> snkRampServo;
    public Source<DoubleSolenoid.Value> srcRampPiston;
    public Sink<DoubleSolenoid.Value> snkRampPiston;

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
    public Source<Double> srcLidarS;
    public Sink<Double> snkLidarF;
    public Sink<Double> snkLidarS;

    // Auto feedback
    public Sink<Double> snkAutoProfile;

    @Override
    public void init() {
        // Ramp
        srcRampPiston = SubsystemRamp.getPiston();
        srcRampServo = SubsystemRamp.getServo();
        snkRampServo = SinkSystems.MOTOR.servo(5);
//        snkRampPiston = SinkSystems.OTHER.dblSolenoid(3, 4);

        // Lift
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
        srcLift = srcLift.inter(srcLiftEnc, // (lift feedback controller)
                Data.inter((v, feedback) -> (feedback >= 0.81D || feedback <= 0.34D)
                        ? 0.5D * v : v));
        snkLift = SinkSystems.MOTOR.talonSrx(5).join(
                SinkSystems.MOTOR.talonSrx(6)
                        .map(Funcs.invertD()));
        snkLiftEnc = SinkSystems.DASH.number("Lift Encoder");
        Sink<Boolean> snkLimit = SinkSystems.DASH.string("Lift At Bottom")
                .map(Data.mapper(Object::toString));
        liftController = new LiftController(srcLiftEnc, snkLift);
        SmartDashboard.putNumber("Lift Setpoint", 0D);

        // Drive
        MotorTuple4 motors = MotorTuple4.ofTalons(2, 3, 1, 4);
        motors.getFrontRight().setInverted(true);
        motors.getRearRight().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(motors);

        // Intake
        srcIntake = SubsystemIntake.get();
        snkIntake = SinkSystems.MOTOR.spark(8).join(
                SinkSystems.MOTOR.spark(9)
                        .map(Funcs.invertD()));

        // LIDAR
        srcLidarF = SourceSystems.SENSOR.lidarLite(0, 38072.7486D);
        snkLidarF = SinkSystems.DASH.number("Front LIDAR");
        srcLidarS = SourceSystems.SENSOR.lidarLite(1, 38072.7486D);
        snkLidarS = SinkSystems.DASH.number("Side LIDAR");

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

        // Control scheme
        SendableChooser<PrimaryDriveScheme> primaryDriveCtrl = new SendableChooser<>();
        for (PrimaryDriveScheme scheme : PrimaryDriveScheme.values()) primaryDriveCtrl.addObject(scheme.name(), scheme);
        primaryDriveCtrl.addDefault(PrimaryDriveScheme.Y_DRIVE.name(), PrimaryDriveScheme.Y_DRIVE);
        SmartDashboard.putData("Primary Drive", primaryDriveCtrl);

        // Camera
        VisionServer.putImageSource("usb cam", VisionServer.getCamera(CameraType.USB, 0));

        // Runmodes
        RobotMode.TELEOP.setOperation(() -> {
            snkRampServo.bind(srcRampServo);
//            snkRampPiston.bind(srcRampPiston);
            snkLift.bind(srcLift);
            snkDrive.bind(primaryDriveCtrl.getSelected().source);
            snkIntake.bind(srcIntake);
            snkLidarF.bind(srcLidarF);
            snkLidarS.bind(srcLidarS);
            snkLiftEnc.bind(srcLiftEnc);
            snkLimit.bind(liftLimit);
            Flow.waitInfinite();
        });

        RobotMode.AUTO.setOperation(() -> {
            liftController.bind();
            StartPos startPos = startPosCtrl.getSelected();
            AutoGoal goal = goalCtrl.getSelected();
            IAutoRoutine routine;
            FieldConfig field = getFieldConfiguration();
            boolean switchSame = field.switchSide == startPos, scaleSame = field.scaleSide == startPos;
            switch (goal) {
                case SWITCH_IF_ON_STARTING_POS_ELSE_SCALE_IF_ON_STARTING_POS:
                    if (switchSame) {
                        routine = new RoutineSwitch();
                    } else if (scaleSame) {
                        routine = new RoutineScaleSame();
                    } else {
                        routine = new RoutineAutoLine();
                    }
                    break;
                case SWITCH_IF_ON_STARTING_POS_ELSE_SCALE:
                    if (switchSame) {
                        routine = new RoutineSwitch();
                    } else if (scaleSame){
                        routine = new RoutineScaleSame();
                    } else {
                        routine = new RoutineScaleOther();
                    }
                    break;
                case SCALE_IF_ON_STARTING_POS_ELSE_SWITCH_IF_ON_STARTING_POS:
                    if (scaleSame) {
                        routine = new RoutineScaleSame();
                    } else if (switchSame) {
                        routine = new RoutineSwitch();
                    } else {
                        routine = new RoutineAutoLine();
                    }
                    break;
                case SCALE_ALWAYS:
                    if (scaleSame) {
                        routine = new RoutineScaleSame();
                    } else {
                        routine = new RoutineScaleOther();
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
            routine.doTheAutoThing(this, startPos.mult);
        });

        RobotMode.TEST.setOperation(() -> {
            liftController.bind();
            double[] s = new double[] {0D};
            liftController.set(0D);
            Flow.whileWaiting(() -> {
                double current = SmartDashboard.getNumber("Lift Setpoint", 0D);
                if (Math.abs(current - s[0]) >= 1e-4) {
                    liftController.set(current);
                    s[0] = current;
                }
            });
            Flow.waitInfinite();
        });

        RobotMode.DISABLED.setOperation(() -> {
            liftController.unbind();
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
        SWITCH_IF_ON_STARTING_POS_ELSE_SCALE_IF_ON_STARTING_POS,
        SWITCH_IF_ON_STARTING_POS_ELSE_SCALE,
        SCALE_IF_ON_STARTING_POS_ELSE_SWITCH_IF_ON_STARTING_POS,
        SCALE_ALWAYS,
        FROM_CENTER_DO_SWITCH,
        DRIVE_ACROSS_AUTO_LINE,
        DO_NOTHING_FOR_30_SECONDS
    }

}
