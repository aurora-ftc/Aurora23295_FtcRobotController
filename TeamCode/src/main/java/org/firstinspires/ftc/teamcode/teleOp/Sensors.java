package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Sensors {
    private static final Logger log = LoggerFactory.getLogger(Sensors.class);

    private final RevColorSensorV3 color = (RevColorSensorV3) hardwareMap.colorSensor.get("colorV3");

    public void init (HardwareMap hwMap) {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Sensor Name", color.getDeviceName());
        telemetry.addLine();
        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());

        telemetry.update();

    }
}
