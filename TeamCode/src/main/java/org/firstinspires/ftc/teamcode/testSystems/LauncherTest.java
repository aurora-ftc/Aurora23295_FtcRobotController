package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;

@TeleOp(name = "LauncherTest", group = "TestModes")
public class LauncherTest extends OpMode {
    private DcMotor launcherMotor;
    private MecanumDrive drive = new MecanumDrive();
    private double power = 0.5;
    @Override
    public void init() {

        launcherMotor = hardwareMap.get(DcMotor.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger >= 0.4) {
            power += 0.05;
        } else if (gamepad1.left_trigger >= 0.4) {
            power -= 0.05;
        }

        if (gamepad1.touchpad) {
            power = 1;
        }

        if (gamepad1.cross) {
            launcherMotor.setPower(power);
        } else if (gamepad1.square) {
            launcherMotor.setPower(-power);
        } else {
            launcherMotor.setPower(0);
        }
    }
}
