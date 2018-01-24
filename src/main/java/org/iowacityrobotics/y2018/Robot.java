package org.iowacityrobotics.y2018;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import org.iowacityrobotics.roboed.data.Data;
import org.iowacityrobotics.roboed.data.sink.Sink;
import org.iowacityrobotics.roboed.data.source.Source;
import org.iowacityrobotics.roboed.robot.Flow;
import org.iowacityrobotics.roboed.robot.IRobotProgram;
import org.iowacityrobotics.roboed.robot.RobotMode;
import org.iowacityrobotics.roboed.subsystem.SinkSystems;
import org.iowacityrobotics.roboed.util.math.Vector4;
import org.iowacityrobotics.roboed.util.robot.MotorTuple4;

import java.util.concurrent.TimeUnit;

public class Robot implements IRobotProgram {

    @Override
    public void init() {
        MotorTuple4 motors = MotorTuple4.ofTalons(1, 4, 3, 6);
        motors.getFrontRight().setInverted(true);
        motors.getRearRight().setInverted(true);
        Sink<Vector4> snkDrive = SinkSystems.DRIVE.mecanum(motors);
        AHRS gyro = new AHRS(SPI.Port.kMXP);
        RobotMode.AUTO.setOperation(() -> {
            Vector4 vec = new Vector4(0, 0.45, 0, 0);
            Source<Vector4> src = Data.source(() -> vec);
            snkDrive.bind(src);
            Flow.waitFor(1, TimeUnit.SECONDS);

            Vector4 vec2 = new Vector4(0, 0, -0.26, 0);
            Source<Vector4> src2 = Data.source(() -> vec2);
            double initialAngle = gyro.getAngle();
            snkDrive.bind(src2);
            Flow.waitUntil(() -> Math.abs(gyro.getAngle() - initialAngle) >= 90);
        });
    }
}
