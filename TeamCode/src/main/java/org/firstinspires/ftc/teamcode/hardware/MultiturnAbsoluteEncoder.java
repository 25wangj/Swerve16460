package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Double.NaN;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.AnalogInput;
public class MultiturnAbsoluteEncoder {
    private AnalogInput encoder;
    private int turns;
    private double offset;
    private double maxDiff;
    private int num = 0;
    private double rot = 0;
    private double lastT = NaN;
    private double lastSpeed = NaN;
    public MultiturnAbsoluteEncoder(AnalogInput encoder, int turns, double offset, double maxDiff) {
        this.encoder = encoder;
        this.turns = turns;
        this.offset = offset;
        this.maxDiff = maxDiff;
    }
    public void update(double t) {
        double curr = ((encoder.getVoltage() - offset) / 3.3 + 1) % 1;
        double delta = 2 * PI * ((curr - rot + 1.5) % 1 - 0.5);
        double speed = delta / (t - lastT);
        if (!Double.isNaN(lastT) && abs(speed - lastSpeed) < maxDiff) {
            num = ((int) round(rot - curr) + num + turns) % turns;
            rot = curr;
        }
        lastT = t;
        lastSpeed = speed;
    }
    public double get() {
        return (num + rot) * 2 * PI / turns;
    }
    public void set(double near) {
        num = ((int)round(near * turns / (2 * PI) - rot) + turns) % turns;
    }
}