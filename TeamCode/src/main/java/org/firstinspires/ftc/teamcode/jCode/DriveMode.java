package org.firstinspires.ftc.teamcode.jCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jCode.SubSystems.DriveController;
import org.firstinspires.ftc.teamcode.jCode.SubSystems.IOController;
import org.firstinspires.ftc.teamcode.jCode.SubSystems.Sorter;

/**
 * <p>Before you change anything...</p>
 * <b>DID YOU READ MY README? </b>
 * <p>thank you for your understanding</p>
 */

@TeleOp(name = "EXPERIMENTAL: DO NOT USE UNLESS YOU KNOW WHAT YOU ARE DOING!", group = "Zeta")
public class DriveMode extends OpMode {
    private final DriveController driveController = new DriveController();
    private final Sorter sortController = new Sorter();
    private final IOController ioController = new IOController();
    //    private final Limelight llController = new Limelight();
    private double rightStickX, rightStickY, leftStickX, leftStickY;
    private boolean dpadUp, dpadDown, dpadLeft, dpadRight;
    private boolean square, circle, triangle, cross;
    private boolean leftBumper, rightBumper;
    private float leftTrigger, rightTrigger;
    private double slow;
    private boolean lockOn;

    @Override
    public void init() {

        driveController.init(hardwareMap, telemetry);
        sortController.init(hardwareMap);
        ioController.init(hardwareMap);
//        llController.init(hardwareMap);
    }

    @Override
    public void loop() {
        inputs();
        logic();
        control();
        updateTelemetry();
    }

    /**
     * Reads all gamepad inputs
     */
    private void inputs() {

        // Analog sticks
        rightStickX = gamepad1.right_stick_x;
        rightStickY = -gamepad1.right_stick_y;
        leftStickX = gamepad1.left_stick_x;
        leftStickY = -gamepad1.left_stick_y;

        // D-Pad
        dpadDown = gamepad1.dpad_down;
        dpadLeft = gamepad1.dpad_left;
        dpadRight = gamepad1.dpad_right;
        dpadUp = gamepad1.dpad_up;

        // Buttons. PS5 + X-Box compatible.
        square = gamepad1.squareWasPressed() || gamepad1.xWasPressed();
        circle = gamepad1.circleWasPressed() || gamepad1.bWasPressed();
        triangle = gamepad1.triangleWasPressed() || gamepad1.yWasPressed();
        cross = gamepad1.crossWasPressed() || gamepad1.aWasPressed();

        // Bumpers and Triggers
        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;
    }

    /**
     * Controls all of the various robot state machines and other logic.
     */
    private void logic() {
        if (rightTrigger > 0.4) slow = rightTrigger * -1.2;
        else slow = 1;

        if (triangle) {
            sortController.switchState();
        }
        if (dpadUp) {
            lockOn = !lockOn;
        }
    }

    /**
     * Calls various repeated methods and causes other command execution
     */
    private void control() {
        if (square) {
            ioController.toggleOuttake();
        }

        if (circle) {
            ioController.toggleIntake();
        }

        if (cross) {
            ioController.push();
        }

        if (dpadLeft) {
            sortController.rotateUp();
        } else if (dpadRight) {
            sortController.rotateDown();
        }

        driveController.driveFieldOriented(leftStickY, leftStickX, rightStickX, slow);
        sortController.periodic();
        ioController.periodic();
//        llController.periodic();
    }

    /**
     * Does exactly what it says on the tin. Updates all of the various subsystem telemetries.
     */
    private void updateTelemetry() {
        driveController.driveTelemetry(telemetry);
        ioController.updateTelemetry(telemetry);
        sortController.updateTelemetry(telemetry);
//        llController.updateTelemetry(telemetry);
    }
}
