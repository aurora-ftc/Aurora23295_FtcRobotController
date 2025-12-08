package org.firstinspires.ftc.teamcode.teleOp.mainOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;

@TeleOp(name = "DriveOpMode", group = "OpModes")
public class DriveOpMode extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    double forward, strafe, rotate, slow;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private boolean endgameRumbleDone;
    private double recenterTime = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initializes hardware
        drive.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {

        matchTimer.reset();
        endgameRumbleDone = false;

    }

    @Override
    public void loop() {

        //Keep Robot still while re-centering ODO
        if (recenterTime > 0) {
            // If 0.25 seconds have passed, end freeze
            if (matchTimer.seconds() - recenterTime >= 0.25) {
                recenterTime = 0; // done freezing
            } else {
                // Still in freeze period: stop motors and skip input processing
                drive.drive(0, 0, 0, 0);
                telemetry.addLine("Recalibrating IMU...");
                telemetry.update();
                return; // skip the rest of loop for now
            }
        }

        //Takes controller inputs
        forward = -1 * gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x * 1.1;

        //Alter driving speed (Left trigger = half speed, right trigger = double Speed)
        if (gamepad1.left_trigger > 0.4) {
            slow = 0.5;
        } else if (gamepad1.right_trigger > 0.4) {
            slow = 2;
        } else {
            slow = 1;
        }

        //Recenter field-centric driving
        //-STAY STILL FOR AT LEAST 0.25 SECONDS WHILE DOING SO FOR ACCURACY-
        if (gamepad1.touchpad) {
            gamepad1.rumbleBlips(2);
            recenterTime = matchTimer.seconds();
            drive.resetOdoHeading();
            return;
        }

        //Rumble controllers in Endgame
        if (matchTimer.seconds() >= 90 && !endgameRumbleDone) {
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            endgameRumbleDone = true;
        }

        /*
        Use this for non field-centric code:
        drive.drive(forward, strafe, rotate, slow);
         */
        drive.driveFieldOriented(forward, strafe, rotate, slow, telemetry);
        
        // All telemetry is now handled in updateTelemetry methods
        drive.updateTelemetry(telemetry, slow);

    }
}
