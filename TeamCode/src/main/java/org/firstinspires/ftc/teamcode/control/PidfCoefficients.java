package org.firstinspires.ftc.teamcode.control;
import java.util.function.ToDoubleFunction;
public class PidfCoefficients {
    public final double kp;
    public final double ki;
    public final double kd;
    public final ToDoubleFunction<double[]> kf;
    public PidfCoefficients(double kp, double ki, double kd, ToDoubleFunction<double[]> kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }
    public PidfCoefficients(double kp, double ki, double kd) {
        this(kp, ki, kd, x -> 0);
    }
}