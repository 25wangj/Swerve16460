package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
public class WrappingPidfController {
    private PidfCoefficients coeffs;
    private double hyst;
    private double setPoint = 0;
    private double e1 = 0;
    private double e2 = 0;
    private double i1 = 0;
    private double i2 = 0;
    private double d1 = 0;
    private double d2 = 0;
    private double val = 0;
    private boolean flipped = false;
    private double lastTime = Double.NaN;
    private double lastE1 = 0;
    private double lastE2 = 0;
    public WrappingPidfController(PidfCoefficients coeffs, double hyst) {
        this.coeffs = coeffs;
        this.hyst = hyst;
    }
    public WrappingPidfController(PidfCoefficients coeffs) {
        this(coeffs, 0);
    }
    public void reset() {
        i1 = 0;
        i2 = 0;
    }
    public void setCoeffs(PidfCoefficients coeffs) {
        this.coeffs = coeffs;
    }
    public void set(double newSetPoint) {
        setPoint = newSetPoint;
    }
    public double get() {
        return val;
    }
    public void update(double time, double... x) {
        e1 = ((setPoint - x[0] + PI) % (2 * PI) + 2 * PI) % (2 * PI) - PI;
        e2 = e1 < 0 ? e1 + PI : e1 - PI;
        if ((flipped && abs(e2) > PI / 2 + hyst) || (!flipped && abs(e1) > PI / 2 + hyst)) {
            flipped = !flipped;
            reset();
        }
        if (!Double.isNaN(lastTime)) {
            double dt = time - lastTime;
            i1 += (e1 + lastE1) * dt / 2;
            i2 += (e2 + lastE2) * dt / 2;
            d1 = (e1 - lastE1) / dt;
            d2 = (e2 - lastE2) / dt;
        }
        val = coeffs.kf.applyAsDouble(x) + (flipped ? coeffs.kp * e2 + coeffs.ki * i2 + coeffs.kd * d2 : coeffs.kp * e1 + coeffs.ki * i1 + coeffs.kd * d1);
        lastTime = time;
        lastE1 = e1;
        lastE2 = e2;
    }
}
