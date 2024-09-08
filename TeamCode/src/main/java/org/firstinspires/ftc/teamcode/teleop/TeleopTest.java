package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.FnCommand;
import org.firstinspires.ftc.teamcode.command.RisingEdgeDetector;
import org.firstinspires.ftc.teamcode.hardware.OmniSwerve;
import org.firstinspires.ftc.teamcode.hardware.ValueStorage;
import org.firstinspires.ftc.teamcode.movement.Pose;
import org.firstinspires.ftc.teamcode.movement.Vec;

@Photon
@TeleOp(name="Test")
public class TeleopTest extends CommandOpMode {
    @Override
    public void initOpMode() {
        ValueStorage.telemetry = telemetry;
        OmniSwerve swerve = new OmniSwerve(hardwareMap);
        register(swerve);
        scheduler.addListener(RisingEdgeDetector.listen(() -> gamepad1.ps, swerve.zero()));
        scheduler.schedule(FnCommand.repeat(t -> {
            if (!scheduler.using(swerve)) {
                Pose v = new Pose(0, 0, 0);
                if (pow(gamepad1.left_stick_x, 2) + pow(gamepad1.left_stick_y, 2) + pow(gamepad1.right_stick_x, 2) > 0.001) {
                    v = new Pose(new Vec(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotate(-swerve.getHeading()), -gamepad1.right_stick_x);
                }
                swerve.drive(v, scale(gamepad1.right_trigger, 0.1, 0.9, 0, 1));
            }
            telemetry.addData("Heading", swerve.getHeading());
        }));
    }
}
