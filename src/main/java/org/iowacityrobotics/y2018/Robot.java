package org.iowacityrobotics.y2018;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.IRobotProgram;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;

public class Robot implements IRobotProgram {

    // Ramp
    Source<Double> srcRampServo;
    Sink<Double> snkRampServo;
    Source<DoubleSolenoid.Value> srcRampPiston;
    Sink<DoubleSolenoid.Value> snkRampPiston;

    // Lift
    Source<Double> srcLift;
    Sink<Double> snkLift;

    // Drive
    Source<Vector4> srcDrive;
    Sink<Vector4> snkDrive;

    // Intake
    Source<Double> srcIntakeFwd;
    Source<Double> srcIntakeRev;
    Sink<Double> snkIntakeFwd;
    Sink<Double> snkIntakeRev;

    @Override
    public void init() {
        // Ramp
        srcRampPiston = SubsystemRamp.getPiston();
        srcRampServo = SubsystemRamp.getServo();
        snkRampServo = SinkSystems.MOTOR.servo(0);
        snkRampPiston = SinkSystems.OTHER.dblSolenoid(0, 0);

        // Lift
        srcLift = SubsystemLift.get();
        snkLift = SinkSystems.MOTOR.talonSrx(0);

        // Drive
        srcDrive = SubsystemDrive.get();
        MotorTuple4 motors = MotorTuple4.ofTalons(1, 4, 3, 6);
        motors.getFrontRight().setInverted(true);
        motors.getRearRight().setInverted(true);
        snkDrive = SinkSystems.DRIVE.mecanum(motors);

        // Intake
        srcIntakeFwd = SubsystemIntake.getForwards();
        srcIntakeRev = SubsystemIntake.getReverse();
        snkIntakeFwd = SinkSystems.MOTOR.talonSrx(0);
        snkIntakeRev = SinkSystems.MOTOR.talonSrx(0);

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
            // TODO Implement
        });
    }
}
