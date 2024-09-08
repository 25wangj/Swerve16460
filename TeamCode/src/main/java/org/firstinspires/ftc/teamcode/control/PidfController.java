package org.firstinspires.ftc.teamcode.control;
public class PidfController {
    private PidfCoefficients coeffs;
    private double setPoint = 0;
    private double e = 0;
    private double i = 0;
    private double d = 0;
    private double f = 0;
    private double lastTime = Double.NaN;
    private double lastE = 0;
    private double val = 0;
    public PidfController(PidfCoefficients coeffs) {
        this.coeffs = coeffs;
    }
    public void reset() {
        i = 0;
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
        e = setPoint - x[0];
        if (!Double.isNaN(lastTime)) {
            double dt = time - lastTime;
            i += (e + lastE) * dt / 2;
            d = (e - lastE) / dt;
        }
        val = coeffs.kp * e + coeffs.ki * i + coeffs.kd * d + coeffs.kf.applyAsDouble(x);
        lastTime = time;
        lastE = e;
    }
    public void derivUpdate(double time, double d, double... x) {
        if (!Double.isNaN(d)) {
            this.d = d;
        }
        e = setPoint - x[0];
        if (!Double.isNaN(lastTime)) {
            i += (e + lastE) * (time - lastTime) / 2;
        }
        val = coeffs.kp * e + coeffs.ki * i + coeffs.kd * d + coeffs.kf.applyAsDouble(x);
        lastTime = time;
        lastE = e;
    }
}