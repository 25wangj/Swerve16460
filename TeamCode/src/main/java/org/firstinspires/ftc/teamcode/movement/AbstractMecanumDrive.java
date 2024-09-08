package org.firstinspires.ftc.teamcode.movement;
import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.control.AsymConstraints;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.control.PidfController;
public abstract class AbstractMecanumDrive extends Drivetrain {
    protected DcMotorEx fr;
    protected DcMotorEx fl;
    protected DcMotorEx br;
    protected DcMotorEx bl;
    private double trackWidth;
    private double ks;
    private double kv;
    private double ka;
    private double strafeMult;
    private PidfController xPid;
    private PidfController yPid;
    private PidfController turnPid;
    public AbstractMecanumDrive(double trackWidth, double ks, double kv, double ka, double strafeMult, PidfCoefficients moveCoeffs, PidfCoefficients turnCoeffs, AsymConstraints moveConstraints, AsymConstraints turnConstraints, boolean auto) {
        super(auto, moveConstraints, turnConstraints);
        this.trackWidth = trackWidth;
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        this.strafeMult = strafeMult;
        xPid = new PidfController(moveCoeffs);
        yPid = new PidfController(moveCoeffs);
        turnPid = new PidfController(turnCoeffs);
    }
    public void setPowers(Vec v, double t) {
        double d = max(abs(v.x) + abs(strafeMult * v.y) + abs(t), 1);
        fr.setPower(offset((v.x + strafeMult * v.y + t) / d, ks));
        fl.setPower(offset((v.x - strafeMult * v.y - t) / d, ks));
        br.setPower(offset((v.x - strafeMult * v.y + t) / d, ks));
        bl.setPower(offset((v.x + strafeMult * v.y - t) / d, ks));
    }
    @Override
    public void follow(double time) {
        TrajectoryState state = traj.state(time);
        Pose acPos = pose();
        Pose acVel = vel();
        Vec locPosError = acPos.vec().combo(1, state.pos.vec(), -1).rotate(-acPos.h);
        Vec locVelError = acVel.vec().combo(1, state.pos.vec(), -1).rotate(-acPos.h);
        Vec locVel = state.vel.vec().rotate(-acPos.h);
        Vec locAccel = state.accel.rotate(-acPos.h);
        xPid.derivUpdate(time, locVelError.x, locPosError.x);
        yPid.derivUpdate(time, locVelError.y, locPosError.y);
        turnPid.derivUpdate(time, acVel.h - state.vel.h, ((acPos.h - state.pos.h) % (2 * PI) + 3 * PI) % (2 * PI) - PI);
        setPowers(new Vec(xPid.get(), yPid.get()).combo(1, locVel, kv).combo(1, locAccel, ka),
                turnPid.get() + kv * state.vel.h * trackWidth);
    }
    private double offset(double a, double b) {
        return a + signum(a) * b;
    }
}
