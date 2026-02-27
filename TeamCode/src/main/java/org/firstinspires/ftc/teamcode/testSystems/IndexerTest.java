//package org.firstinspires.ftc.teamcode.testSystems;
//
//import static org.firstinspires.ftc.teamcode.teleOp.Constants.BLUE_SIDE;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.teleOp.Constants;
//import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
//import org.firstinspires.ftc.teamcode.teleOp.subSystems.Indexer;
//import org.firstinspires.ftc.teamcode.teleOp.subSystems.Intake;
//import org.firstinspires.ftc.teamcode.teleOp.subSystems.Launcher;
//
//@TeleOp
//public class IndexerTest extends OpMode {
//    private Indexer indexer;
//    private Intake intake;
//    private Launcher launcher;
//    private MecanumDrive drive;
//    private double forward, strafe, rotate, slow;
//    private double lastHeading = 0;
//    private boolean projHeadingCalculated, rightTrigger;
//    private final ElapsedTime PIDTimer = new ElapsedTime();
//
//    @Override
//    public void init() {
//        indexer = new Indexer();
//        intake = new Intake();
//        launcher = new Launcher();
//        drive = new MecanumDrive();
//
//        indexer.init(hardwareMap);
//        intake.init(hardwareMap);
//        launcher.init(hardwareMap);
//        drive.init(hardwareMap, telemetry);
//
//        Pose2D goalPose;
//        if (BLUE_SIDE) {
//            goalPose = Constants.Pose.GOAL_POSE_BLUE;
//        } else {
//            goalPose = Constants.Pose.GOAL_POSE_RED;
//        }
//
//        drive.initTracker(goalPose, false);
//
//        launcher.disableAutoPower();
//
//        PIDTimer.reset();
//    }
//
//    @Override
//    public void loop() {
//
//        //Controller 1
//        forward = MecanumDrive.smoothDrive(-1 * gamepad1.left_stick_y);
//        strafe = MecanumDrive.smoothDrive(gamepad1.left_stick_x);
//
//        if (!drive.trackGoalOn) {
//            if (Math.abs(gamepad1.right_stick_x) > 0.03) {
//                rotate = MecanumDrive.smoothDrive(gamepad1.right_stick_x);
//                lastHeading = drive.getOdoHeading(AngleUnit.RADIANS);
//                projHeadingCalculated = false;
//                PIDTimer.reset();
//            } else if (PIDTimer.milliseconds() > 160) {
//                if (!projHeadingCalculated) {
//                    lastHeading = drive.getOdoHeading(AngleUnit.RADIANS);
//                    projHeadingCalculated = true;
//                }
//                rotate = drive.headingPID(lastHeading);
//            } else {
//                rotate = 0;
//            }
//            drive.driveFieldOriented(forward, strafe, rotate, slow);
//        } else {
//            drive.trackGoal(forward, strafe, slow);
//        }
//
//        if (gamepad1.left_trigger >= 0.4)
//            slow = 0.7;
//        else if (gamepad1.right_trigger >= 0.4)
//            slow = 1.25;
//        else
//            slow = 1;
//
//        if (gamepad1.squareWasPressed())
//            intake.toggleIn();
//        else if (gamepad1.circleWasPressed())
//            intake.toggleOut();
//
//        //Controller 2
//
//        if (gamepad2.left_bumper)
//            launcher.angleUp();
//        else if (gamepad2.right_bumper)
//            launcher.angleDown();
//
//        if (gamepad2.dpadLeftWasPressed())
//            indexer.clockwise();
//        else if (gamepad2.dpadRightWasPressed())
//            indexer.counterClockwise();
//
//        if (gamepad2.dpadUpWasPressed())
//            launcher.stepUpPower();
//        else if (gamepad2.dpadDownWasPressed())
//            launcher.stepDownPower();
//
//        if (gamepad2.triangleWasPressed()) {
//            indexer.toggleState();
//        }
//
//        if (gamepad2.crossWasPressed())
//            indexer.shoot();
//
//        if (gamepad2.circleWasPressed())
//            launcher.toggleLauncher();
//
//        //Periodic
//
//        indexer.periodic(intake);
//        intake.periodic();
//
//        launcher.updateLauncher(drive.getDistanceFromGoal(), hardwareMap);
//        drive.driveFieldOriented(forward, strafe, rotate, slow);
//
//        //Telemetry
//
//        drive.log(telemetry, slow);
//        indexer.log(telemetry);
//        launcher.log(telemetry);
//
//    }
//}
