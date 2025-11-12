package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleOp.subSystems.BallColorDetection;

@TeleOp(name = "ColorSensorTest",group = "TestModes")
public class ColorSensorTest extends OpMode {
    BallColorDetection colorSensor = new BallColorDetection();
    BallColorDetection.DetectedColor detectedColor;
    @Override
    public void init() {
        colorSensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        detectedColor = colorSensor.getDetectedColor();
        telemetry.addData("color", detectedColor);
        colorSensor.debugTelemetry(telemetry);
        telemetry.update();
    }
}
