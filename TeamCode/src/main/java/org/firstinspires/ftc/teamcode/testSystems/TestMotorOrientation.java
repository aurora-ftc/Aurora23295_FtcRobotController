package org.firstinspires.ftc.teamcode.testSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;

@TeleOp(name = "TestMotorOrientation", group = "TestModes")
public class TestMotorOrientation extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();

        telemetry.addData("frontLeftMotor Type", frontLeftMotor.getMotorType());
        telemetry.addData("frontRightMotor Type", frontRightMotor.getMotorType());
        telemetry.addData("backRightMotor Type", backRightMotor.getMotorType());
        telemetry.addData("backLeftMotor Type", backLeftMotor.getMotorType());
        telemetry.addLine();

        telemetry.addData("frontLeftMotor Version", frontLeftMotor.getVersion());
        telemetry.addData("frontRightMotor Version", frontRightMotor.getVersion());
        telemetry.addData("backRightMotor Version", backRightMotor.getVersion());
        telemetry.addData("backLeftMotor Version", backLeftMotor.getVersion());

    }

    @Override
    public void loop() {

        if (gamepad1.square) {frontLeftMotor.setPower(1);} else {frontLeftMotor.setPower(0);}
        if (gamepad1.triangle) {frontRightMotor.setPower(1);} else {frontRightMotor.setPower(0);}
        if (gamepad1.circle) {backRightMotor.setPower(1);} else {backRightMotor.setPower(0);}
        if (gamepad1.cross) {backLeftMotor.setPower(1);} else {backLeftMotor.setPower(0);}

        telemetry.addData("Square", "Front Left Motor");
        telemetry.addData("Triangle", "Front Right Motor");
        telemetry.addData("Circle", "Back Right Motor");
        telemetry.addData("Cross", "Back Left Motor");
        telemetry.addLine();

        telemetry.addData("frontLeftMotor Power", frontLeftMotor.getPower());
        telemetry.addData("frontRightMotor Power", frontRightMotor.getPower());
        telemetry.addData("backRightMotor Power", backRightMotor.getPower());
        telemetry.addData("backLeftMotor Power", backLeftMotor.getPower());
        telemetry.addLine();

    }
}
