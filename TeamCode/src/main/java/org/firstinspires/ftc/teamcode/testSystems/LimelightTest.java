package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LimelightLocalization;

@TeleOp
public class LimelightTest extends OpMode {
    LimelightLocalization ll = new LimelightLocalization();
    MecanumDrive drive = new MecanumDrive();
    @Override
    public void init() {
        ll.init(hardwareMap, 0);
        drive.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        double heading = drive.getOdoHeading(AngleUnit.RADIANS);
        telemetry.addData("2dLocation",ll.get2DLocation(heading));
        telemetry.addData("3dLocation",ll.getLocation(heading));
        telemetry.update();
        if (gamepad1.touchpadWasPressed()) drive.resetOdoHeading(telemetry);
    }
}
