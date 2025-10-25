package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleOp.launchSubSystem.LaunchSystem;

@TeleOp(name = "DriveLaunchMode", group = "OpModes")
public class DriveLaunchMode extends OpMode {
    private MecanumDrive drive = new MecanumDrive();
    private ElapsedTime matchTime = new ElapsedTime();
    private final double[] powerSteps = {0.6, 0.67, 0.72, 1.0};
    LaunchSystem launchSystem = new LaunchSystem();
    double startWait = 0.0;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean liftDown = true;
    private double slow = 1;
    private boolean endgameRumbleDone;
    private double recenterTime = 0;

    @Override
    public void init() {
        //Initialize hardware
        drive.init(hardwareMap, telemetry);
        //NOTE: The MIN is for the UPPER limit, the MAX if the the LOWER limit
        launchSystem.init(0.10, 0.24, powerSteps, hardwareMap, telemetry);
    }

    @Override
    public void start() {
        matchTime.reset();
        endgameRumbleDone = false;
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
    }

    @Override
    public void loop() {

        //Keep Robot still while re-centering ODO
        if (recenterTime > 0) {
            // If 0.25 seconds have passed, end freeze
            if (matchTime.seconds() - recenterTime >= 0.25) {
                recenterTime = 0; // done freezing
            } else {
                // Still in freeze period: stop motors and skip input processing
                drive.drive(0, 0, 0, 0, telemetry);
                telemetry.addLine("Recalibrating IMU...");
                telemetry.update();
                return; // skip the rest of loop for now
            }
        }

        //Take controller inputs
        double forward = -1 * gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * 1.1;

        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else if (gamepad1.right_trigger > 0.4) {
            slow = 0.25;
        }else {
            slow = 1;
        }


        if (matchTime.seconds() >= 90 && !endgameRumbleDone) {
            gamepad1.rumble(500);
            endgameRumbleDone = true;
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            launchSystem.stepUpPower();
        } else if (gamepad1.dpad_down && !lastDpadDown) {
            launchSystem.stepDownPower();
        }

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.triangleWasPressed()) {
            launchSystem.toggleLauncher();
        }

        if (gamepad1.squareWasPressed()) {
            launchSystem.toggleIntake();
        }

        if (gamepad1.circleWasPressed()) {
            launchSystem.toggleIntakeReverse();
        }

        if (gamepad1.crossWasPressed()) {
            launchSystem.liftUp();
            startWait = matchTime.milliseconds();
            liftDown = false;
        }

        if (!liftDown && matchTime.milliseconds() >= startWait + 100) {
            launchSystem.liftDown();
            liftDown = true;
            launchSystem.intakeBlipReset();
        }

        if (gamepad1.touchpad) {
            gamepad1.rumbleBlips(2);
            recenterTime = matchTime.seconds();
            drive.OdoReset(telemetry);
            return; //Return to top of loop
        }

        //Crowding
        telemetry.addData("forward", forward);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addData("speed", slow);
        launchSystem.updateTelemetry(telemetry);

        launchSystem.intakeBlipLoop();
        launchSystem.updateLauncher();
        drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);
    }
}