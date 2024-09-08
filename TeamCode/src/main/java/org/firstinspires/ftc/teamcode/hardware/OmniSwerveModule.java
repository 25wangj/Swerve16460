package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;
public class OmniSwerveModule implements Subsystem {
    private double hyst = 0.1;
    private double pidHyst = 0.1;
    private double zeroSpeed = 1.5;
    private double steps = 20;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private CRServo turn;
    private MultiturnAbsoluteEncoder encoder;
    private DigitalChannel zero;
    private WrappingPidfController pid;
    private Vec pos;
    private int sgn = 1;
    public OmniSwerveModule(DcMotorEx motor1, DcMotorEx motor2, CRServo turn, MultiturnAbsoluteEncoder encoder, DigitalChannel zero, PidfCoefficients coeffs, Vec pos) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.turn = turn;
        this.encoder = encoder;
        this.zero = zero;
        this.pid = new WrappingPidfController(coeffs, pidHyst);
        this.pos = pos;
    }
    public Pose invKin(Pose v, double ratio) {
        if (v.zero()) {
            return new Pose(0, 0, 0);
        }
        Vec rot = pos.rotate(PI / 2);
        Vec v1 = v.vec().combo(1, rot, v.h);
        Vec v2 = v.vec().combo(1, rot, -v.h);
        double d1 = v1.angle();
        double d2 = v2.angle();
        double a1 = v1.norm();
        double a2 = v2.norm();
        double t = atan2(a1*a1*sin(2*d1) + a2*a2*sin(2*d2), a1*a1*cos(2*d1) + a2*a2*cos(2*d2)) / 2;
        double d = acos(1 / ratio);
        Vec posDir = pos.rotate(hyst*signum(v.h)).normalize();
        if (abs(Vec.dir(t - sgn*d).dot(posDir)) + hyst < abs(Vec.dir(t + sgn*d).dot(posDir))) {
            sgn *= -1;
        }
        t += sgn*d;
        return new Pose(a1*ratio*cos(t - d1), a2*ratio*cos(t - d2), t);
    }
    public Command zero() {
        double[] start = {0, 0};
        return new FnCommand(t -> {
            start[0] = t;
            start[1] = encoder.get();
            motor1.setPower(0);
            motor2.setPower(0);
        }, t -> pid.set(zeroSpeed * (t - start[0]) + start[1]), (t, b) -> {
            encoder.set(0);
            pid.set(pos.angle() + PI/2);
        }, t -> !zero.getState());
    }
    public void set(Pose v, double ratio, double ang) {
        if (v.zero()) {
            motor1.setPower(0);
            motor2.setPower(0);
        } else {
            Vec rot = pos.rotate(PI / 2);
            Vec v1 = v.vec().combo(1, rot, v.h);
            Vec v2 = v.vec().combo(1, rot, -v.h);
            Vec dir = Vec.dir(encoder.get());
            motor1.setPower(round(v1.dot(dir) * ratio * steps) / steps);
            motor2.setPower(round(v2.dot(dir) * ratio * steps) / steps);
            pid.set(ang);
        }
    }
    @Override
    public void update(double time, boolean active) {
        encoder.update(time);
        pid.update(time, encoder.get());
        turn.setPower(round(pid.get() * steps) / steps);
    }
}
