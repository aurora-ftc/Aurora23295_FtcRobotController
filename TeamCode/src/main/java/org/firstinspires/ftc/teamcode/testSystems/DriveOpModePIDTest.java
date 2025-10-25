package org.firstinspires.ftc.teamcode.testSystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;

@Config
@TeleOp(name = "DriveOpModePIDTest", group = "TestModes")
public class DriveOpModePIDTest extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate, slow;
    double pidP = 5;
    double pidI = 0.0;
    double pidD = 0.0;

    @Override
    public void init() {

        drive.init(hardwareMap, telemetry);
        dashboard.isEnabled();

    }

    @Override
    public void loop() {

        forward = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else if (gamepad1.right_trigger > 0.4) {
            slow = 2;
        } else {
            slow = 1;
        }

        if (gamepad1.dpad_up) {
            drive.OdoReset(telemetry);
        }

        telemetry.addData("Elapsed Time", getRuntime());
        telemetry.addLine();

        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("speed", slow);

        //TODO: CHANGE 0 TO ROTATE ONCE CODE WORKS
        drive.turnToHeading(0, slow, telemetry, pidP, pidI, pidD);
    }
}
