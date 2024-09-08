package org.firstinspires.ftc.teamcode.hardware;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.ParCommand;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.control.PidfCoefficients;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;
public class OmniSwerve implements Subsystem {
    private int turns = 4;
    private double offsetR = 2.058;
    private double offsetL = 0.673;
    private Vec posR = new Vec(3.092, -3.092);
    private Vec posL = new Vec(3.092, 3.092);
    private double maxR = max(posR.norm(), posL.norm());
    private double minRatio = 1.05;
    private double maxRatio = 2;
    private double maxDiff = 100;
    private double interval = 0.1;
    private OmniSwerveModule moduleR;
    private OmniSwerveModule moduleL;
    public MultiturnAbsoluteEncoder encoderR;
    public MultiturnAbsoluteEncoder encoderL;
    private IMU imu;
    private ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    private double heading;
    private double offset;
    private double next;
    private PidfCoefficients coeffs = new PidfCoefficients(1, 0.5, 0.01);
    public OmniSwerve(HardwareMap hwMap) {
        DcMotorEx fr = hwMap.get(DcMotorEx.class, "fr");
        DcMotorEx fl = hwMap.get(DcMotorEx.class, "fl");
        DcMotorEx br = hwMap.get(DcMotorEx.class, "br");
        DcMotorEx bl = hwMap.get(DcMotorEx.class, "bl");
        CRServo turnR = hwMap.get(CRServo.class, "turnR");
        CRServo turnL = hwMap.get(CRServo.class, "turnL");
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        turnR.setDirection(DcMotorSimple.Direction.REVERSE);
        turnL.setDirection(DcMotorSimple.Direction.REVERSE);
        DigitalChannel zeroR = hwMap.get(DigitalChannel.class, "zeroR");
        DigitalChannel zeroL = hwMap.get(DigitalChannel.class, "zeroL");
        AnalogInput absR = hwMap.get(AnalogInput.class, "encoderR");
        AnalogInput absL = hwMap.get(AnalogInput.class, "encoderL");
        encoderR = new MultiturnAbsoluteEncoder(absR, turns, offsetR, maxDiff);
        encoderL = new MultiturnAbsoluteEncoder(absL, turns, offsetL, maxDiff);
        moduleR = new OmniSwerveModule(fr, bl, turnR, encoderR, zeroR, coeffs, posR);
        moduleL = new OmniSwerveModule(fl, br, turnL, encoderL, zeroL, coeffs, posL);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
    }
    public Command zero() {
        return new ParCommand(moduleR.zero(), moduleL.zero(), FnCommand.once(t -> offset = heading, this));
    }
    public void drive(Pose v, double ratio) {
        Pose vn = new Pose(v.vec(), v.h / maxR);
        double sum = v.vec().norm() + abs(v.h);
        double nMin = scale(abs(v.h) / sum, 1, 0, 1, minRatio);
        double rn = clip(min(v.vec().norm() + abs(v.h), 1) * scale(ratio, 0, 1, nMin, maxRatio), nMin, maxRatio);
        System.out.println(rn);
        Pose powR = moduleR.invKin(vn, rn);
        Pose powL = moduleL.invKin(vn, rn);
        double scale = 1 / max(1, max(max(abs(powR.x), abs(powR.y)), max(abs(powL.x), abs(powL.y))));
        vn = new Pose(vn.vec().mult(scale), vn.h * scale);
        moduleR.set(vn, rn, powR.h);
        moduleL.set(vn, rn, powL.h);
    }
    public double getHeading() {
        return heading - offset;
    }
    @Override
    public void update(double time, boolean active) {
        if (time > next) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            next += interval;
        }
        moduleR.update(time, active);
        moduleL.update(time, active);
    }
}
