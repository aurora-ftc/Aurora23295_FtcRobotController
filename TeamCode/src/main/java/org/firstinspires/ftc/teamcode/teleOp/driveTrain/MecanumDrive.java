package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.teamcode.teleOp.util.DcMotorGroup;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;

import java.util.Locale;

public class MecanumDrive {

    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private GoBildaPinpointDriverRR odo;
    private IMU imu;

    public DcMotorGroup driveMotors;
    private PIDController headingPID;

    private double newForward, newStrafe, theta;
    private String botPose;


    public void init(HardwareMap hwMap, Telemetry telemetry) {

        //Hardware Mapping
        frontLeftMotor = hwMap.get(DcMotorEx.class, Constants.HardwareConfig.frontLeftDriveMotor);
        frontRightMotor = hwMap.get(DcMotorEx.class, Constants.HardwareConfig.frontRightDriveMotor);
        backLeftMotor = hwMap.get(DcMotorEx.class, Constants.HardwareConfig.backLeftDriveMotor);
        backRightMotor = hwMap.get(DcMotorEx.class, Constants.HardwareConfig.backRightDriveMotor);

        driveMotors = new DcMotorGroup(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        odo = hwMap.get(GoBildaPinpointDriverRR.class, Constants.HardwareConfig.odo);

        imu = hwMap.get(IMU.class, Constants.HardwareConfig.imu);

        //Drive Motor Spin Directions
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //Run W/out encoder
        driveMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Brake at 0 Power
        driveMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveMotors.setPower(0);

        //Current Bot Offsets: -41, 0, MM (11/13)
        odo.setOffsets(Constants.odoXOffsetMM, Constants.odoYOffsetMM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriverRR.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //Current Bot Directions: FORWARD, REVERSED (11/13)
        odo.setEncoderDirections(Constants.odoXDirection, Constants.odoYDirection);

        //Calibrate ODO
        odo.resetPosAndIMU();

        headingPID = new PIDController(Constants.driveKp, Constants.driveKi, Constants.driveKd); // tune these values
        headingPID.setTarget(Constants.initialPoseBlue.getHeading(AngleUnit.RADIANS));// default goalPose heading = 0 degrees
        headingPID.previousTime = System.nanoTime() / 1e9;

        String PIDData = String.format(Locale.US, "{KP: %.3f, KI: %.3f, KD: %.3f}",
                Constants.driveKp, Constants.driveKi, Constants.driveKd);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset (Inches)", odo.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Y offset (Inches)", odo.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Odo Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Odo Heading Scalar", odo.getYawScalar());
        telemetry.addData("PID Settings", PIDData);
        telemetry.update();

    }

    public void drive(double forward, double strafe, double rotate, double slow) {
        double frontLeftPower = forward + strafe - rotate;
        double backLeftPower = forward - strafe - rotate;
        double frontRightPower = forward - strafe + rotate;
        double backRightPower = forward + strafe + rotate;

        double maxPower = 1.0;
        double maxSpeed = Constants.maxDriveSpeed;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(slow * maxSpeed * (frontLeftPower / maxPower));
        frontRightMotor.setPower(slow * maxSpeed * (frontRightPower / maxPower));
        backLeftMotor.setPower(slow * maxSpeed * (backLeftPower / maxPower));
        backRightMotor.setPower(slow * maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldOriented(double forward, double strafe, double rotate, double slow, Telemetry telemetry) {

        //Converts X, Y coordinates to polar coordinates
        theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        updateOdoHeading();

        //Recalculates heading based on current position
        double heading = getOdoHeading(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - heading);

        //Converts values back to X, Y coordinates from polar
        newForward = r * Math.sin(theta);
        newStrafe = r * Math.cos(theta);

        //Telemetry
        Pose2D pos = odo.getPosition();
        botPose = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.DEGREES));

        if (Constants.fieldCentric)
            drive(newForward, newStrafe, rotate, slow);
        else
            drive(forward, strafe, rotate, slow);
    }

    public double headingPID(double targetHeading) {

        headingPID.setTarget(targetHeading);

        updateOdoHeading();

        double currentHeading = getOdoHeading(AngleUnit.RADIANS);
        double time = System.nanoTime() / 1e9; //Seconds

        double output = headingPID.calculateOutputPID(currentHeading, time, true);

        //Graphs values for tuning
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("targetDriveD", headingPID.target);
        packet.put("currentDrive", headingPID.current);
        packet.put("outputDrive", headingPID.output);

        //Crucial line: Sends data to FTC Dash
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        return -output; //trust on the negative
    }

    public void debugTelemetry(Telemetry tele, double slow) {
        tele.addData("Speed Modifier", slow);
        tele.addData("New Forward", newForward);
        tele.addData("New Strafe", newStrafe);
        tele.addData("Theta (Radians)", theta);
        tele.addData("Odo Status", odo.getDeviceStatus());
        tele.addData("Pinpoint Frequency", odo.getFrequency()); //prints the current refresh rate of the Pinpoint
        tele.addLine();
        tele.addData("Position", botPose);
        tele.addLine();
        tele.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));
        tele.addLine();
    }

    public void compTelemetry(Telemetry tele, double slow) {
        tele.addData("Odo Status", odo.getDeviceStatus());
        tele.addLine();
        tele.addData("Position", botPose);
        tele.addLine();
        tele.addData("Speed Modifier", slow == Constants.slowSpeedLT
                ? ("Slow(" + Constants.slowSpeedLT + ")")
                : "Normal (1.0)");
        tele.addLine();
    }

    public void setPIDTargetHeading(double targetHeading) {
        headingPID.setTarget(targetHeading);
    }

    public void resetOdoHeading(Telemetry tele) {

        //Resets Heading and Position -STAY STILL FOR AT LEAST 0.25 SECONDS WHILE DOING SO FOR ACCURACY-
        odo.resetYaw();
        updateOdoHeading();
        tele.update();

    }

    public void resetOdoPosition(Telemetry tele) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0 , 0,
                AngleUnit.RADIANS, 0));
        updateOdo();
        tele.update();
    }

    public void updateOdo() {odo.update();}

    public void updateOdoHeading() {
        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
    }

    public void setOdoPosition(Pose2D pose) {
        odo.setPosition(pose);
    }

    public double getOdoHeading(AngleUnit angleUnit) {
        odo.update(GoBildaPinpointDriverRR.ReadData.ONLY_UPDATE_HEADING);
        return odo.getPosition().getHeading(angleUnit);
    }

    public Pose2D getOdoPosition() {
        odo.update();
        return odo.getPosition();
    }

    public double getOdoX(DistanceUnit distanceUnit) {
        odo.update();
        return odo.getPosition().getX(distanceUnit);
    }

    public double getOdoY(DistanceUnit distanceUnit) {
        odo.update();
        return odo.getPosition().getY(distanceUnit);
    }

    public static double smoothDrive(double input) {
        double output = 0.3 * Math.tan(input * 1.2792);
        return output;
    }

    public class GoalTracker {
        private Pose2D goalPose;
        //Keep this public:
        public boolean trackGoalOn;

        public GoalTracker(Pose2D goalPose, boolean trackGoalOn) {
            this.goalPose = goalPose;
            this.trackGoalOn = trackGoalOn;
        }

        public void trackGoal(Telemetry tele, double forward, double strafe, double slow) {
            updateOdo();

            double x = getOdoX(DistanceUnit.INCH);
            double y = getOdoY(DistanceUnit.INCH);

            double deltaY = goalPose.getY(DistanceUnit.INCH) - y;
            double deltaX = goalPose.getX(DistanceUnit.INCH) - x;

            double thetaGoal = AngleUnit.normalizeRadians(Math.atan2(deltaY, deltaX));

            if (Constants.debug) {
                tele.addData("deltaY", deltaY);
                tele.addData("deltaX", deltaX);
                tele.addData("x", y);
                tele.addData("x", x);
                tele.addData("thetaGoal", thetaGoal);
            }

            double output = headingPID(thetaGoal);

            driveFieldOriented(forward, strafe, output, slow, tele);

        }

        public void toggleTrackGoal() {trackGoalOn = !trackGoalOn;}

        public double getDistanceFromGoal() {
            updateOdo();

            double x = getOdoX(DistanceUnit.INCH);
            double y = getOdoY(DistanceUnit.INCH);

            double deltaY = goalPose.getY(DistanceUnit.INCH) - y;
            double deltaX = goalPose.getX(DistanceUnit.INCH) - x;

            double distance = Math.hypot(deltaY, deltaX);

            return distance;
        }
    }
}