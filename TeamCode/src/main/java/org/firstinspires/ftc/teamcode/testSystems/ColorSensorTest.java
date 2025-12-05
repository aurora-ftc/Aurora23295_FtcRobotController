package org.firstinspires.ftc.teamcode.testSystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallColorDetection;

@TeleOp(name = "ColorSensorTest",group = "TestModes")
public class ColorSensorTest extends OpMode {
    BallColorDetection bottomCS = new BallColorDetection();
    BallColorDetection rightCS = new BallColorDetection();
    BallColorDetection leftCS = new BallColorDetection();

    BallColorDetection.DetectedColor bottomColor, rightColor, leftColor;
    @Override
    public void init() {
        bottomCS.init(hardwareMap, HWMap.CS_BOTTOM);
        rightCS.init(hardwareMap, HWMap.CS_RIGHT);
        leftCS.init(hardwareMap, HWMap.CS_LEFT);
    }

    @Override
    public void loop() {
        bottomColor = bottomCS.getDetectedColor();
        rightColor = rightCS.getDetectedColor();
        leftColor = leftCS.getDetectedColor();

        telemetry.addData("Bottom Color", bottomColor);
        telemetry.addData("Right Color", rightColor);
        telemetry.addData("Left Color", leftColor);

        telemetry.addLine();

        telemetry.addData("Bottom Distance", bottomCS.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Distance", rightCS.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", leftCS.getDistance(DistanceUnit.INCH));

        telemetry.update();
    }
}
