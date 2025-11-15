package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teleOp.util.ConstantConfig;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;

import java.util.Locale;

public class MecanumDrive {
    String data;
    double newForward, newStrafe, theta;
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private GoBildaPinpointDriverRR odo;
    private Pose2D goalPose;
    public boolean trackGoalOn;
    private IMU imu;
    private PIDController headingPID;  // PID controller for heading

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        //Hardware Mapping
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");

        odo = hwMap.get(GoBildaPinpointDriverRR.class, "odo");

        imu = hwMap.get(IMU.class, "imu");

        //Drive Motor Spin Directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Brake at 0 Power
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        //New Bot Offsets
        //odo.setOffsets(-0.5, 0, DistanceUnit.INCH);
        //odo.setOffsets(-0.4, 3.6, DistanceUnit.INCH);
        odo.setOffsets(-41, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriverRR.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //Meet 0 Bot Directions: FORWARD, FORWARD
        odo.setEncoderDirections(GoBildaPinpointDriverRR.EncoderDirection.FORWARD,
                GoBildaPinpointDriverRR.EncoderDirection.REVERSED);

        //Calibrate ODO
        odo.resetPosAndIMU();

        headingPID = new PIDController(ConstantConfig.driveKp, ConstantConfig.driveKi, ConstantConfig.driveKd); // tune these values
        headingPID.setTarget(Math.PI / 2.0); //default goalPose heading = 0 degrees
        headingPID.previousTime = System.nanoTime() / 1e9;

        String PIDData = String.format(Locale.US, "{KP: %.3f, KI: %.3f, KD: %.3f}",
                ConstantConfig.driveKp, ConstantConfig.driveKi, ConstantConfig.driveKd);

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
        double maxSpeed = 1.0;

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

        updateOdo();

        //Recalculates heading based on current position
        double heading = getOdoHeading(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - heading);

        //Converts values back to X, Y coordinates from polar
        newForward = r * Math.sin(theta);
        newStrafe = r * Math.cos(theta);

        //Telemetry
        Pose2D pos = odo.getPosition();
        data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.DEGREES));

        this.drive(newForward, newStrafe, rotate, slow);

    }

    public double headingPID(double targetHeading) {

        headingPID.setTarget(targetHeading);

        updateOdo();

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

    public void debugTelemetry(Telemetry telemetry, double slow) {
        telemetry.addData("Speed Modifier", slow);
        telemetry.addData("New Forward", newForward);
        telemetry.addData("New Strafe", newStrafe);
        telemetry.addData("Theta (Radians)", theta);
        telemetry.addData("Odo Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints the current refresh rate of the Pinpoint
        telemetry.addLine();
        telemetry.addData("Position", data);
        telemetry.addLine();
        telemetry.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
    }

    public void setPIDTargetHeading(double targetHeading) {
        headingPID.setTarget(targetHeading);
    }

    public void resetOdoHeading() {

        //Resets Heading and Position -STAY STILL FOR AT LEAST 0.25 SECONDS WHILE DOING SO FOR ACCURACY-
        odo.resetYaw();
        odo.update(GoBildaPinpointDriverRR.ReadData.ONLY_UPDATE_HEADING);

    }

    public void resetOdoPosition(Telemetry telemetry) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,
                AngleUnit.RADIANS, 0));
    }

    public void updateOdo() {
        odo.update();
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

    public void initTracker(Pose2D goalPose, boolean trackGoalOn) {
        this.goalPose = goalPose;
        this.trackGoalOn = trackGoalOn;
    }

    public void drawRobot(Canvas field) {
        Pose2D pose = getOdoPosition();

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        double robotSize = 9;
        double half = robotSize / 2;

        double[][] points = {
                {half, 0},
                {half, half},
                {half, -half},
        };

        double[][] transformed = new double[3][2];
        for (int i = 0; i < 3; i++) {
            transformed[i][0] = x + points[i][0] * Math.cos(heading) - points[i][1] * Math.sin(heading);
            transformed[i][1] = y + points[i][0] * Math.sin(heading) + points[i][1] * Math.cos(heading);
        }

        field.strokePolyline(
                new double[]{transformed[0][0], transformed[1][0], transformed[2][0], transformed[0][0]},
                new double[]{transformed[0][1], transformed[1][1], transformed[2][1], transformed[0][1]}
        );
    }

    public void trackGoal(Telemetry tele, double forward, double strafe, double slow) {
        updateOdo();

        double x = this.getOdoX(DistanceUnit.INCH);
        double y = this.getOdoY(DistanceUnit.INCH);

        double deltaY = goalPose.getY(DistanceUnit.INCH) - y;
        double deltaX = goalPose.getX(DistanceUnit.INCH) - x;

        double thetaGoal = AngleUnit.normalizeRadians(Math.atan2(deltaY, deltaX));

        if (ConstantConfig.debug) {
            tele.addData("deltaY", deltaY);
            tele.addData("deltaX", deltaX);
            tele.addData("x", y);
            tele.addData("x", x);
            tele.addData("thetaGoal", thetaGoal);
        }

        double output = headingPID(thetaGoal);

        driveFieldOriented(forward, strafe, output, slow, tele);
    }

    public double getDistanceFromGoal() {
        updateOdo();

        double x = getOdoX(DistanceUnit.INCH);
        double y = getOdoY(DistanceUnit.INCH);

        double deltaY = goalPose.getY(DistanceUnit.INCH) - y;
        double deltaX = goalPose.getX(DistanceUnit.INCH) - x;

        double distance = Math.hypot(deltaY, deltaX);

        return distance;
    }

    public void activateTrackGoal() {
        trackGoalOn = true;
    }

    public void deactivateTrackGoal() {
        trackGoalOn = false;
    }

    public void toggleTrackGoal() {
        trackGoalOn = !trackGoalOn;
    }

    public double smoothDrive(double input) {
        return 0.3 * Math.tan(input * 1.2792);
    }
    public void updateTelemetry(Telemetry telemetry, double slow) {

        TelemetryPacket packet = new TelemetryPacket();
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        updateOdo();

        Pose2D pos = odo.getPosition();
        double headingDeg = getOdoHeading(AngleUnit.DEGREES);

        multiTelemetry.addLine("===== Mecanum Drive Telemetry =====");
        multiTelemetry.addData("OdoStatus", odo.getDeviceStatus());
        multiTelemetry.addData("Pinpoint Frequency", odo.getFrequency());
        multiTelemetry.addData("X (in)", pos.getX(DistanceUnit.INCH));
        multiTelemetry.addData("Y (in)", pos.getY(DistanceUnit.INCH));
        multiTelemetry.addData("Heading (deg)", headingDeg);

        Canvas field = packet.fieldOverlay();

        drawRobot(field);

        multiTelemetry.addData("Speed Modifier",
                slow == 0.35 ? "Slow (0.35)" : String.format(Locale.US, "Normal (%.2f)", slow));

        multiTelemetry.addData("Forward (calc)", newForward);
        multiTelemetry.addData("Strafe (calc)", newStrafe);
        multiTelemetry.addData("Theta (rad)", theta);

        multiTelemetry.addData("New Forward", newForward);
        multiTelemetry.addData("New Strafe", newStrafe);
        multiTelemetry.addData("Theta (Radians)", theta);

        if (trackGoalOn && goalPose != null) {
            multiTelemetry.addLine("--- Goal Tracking ---");
            multiTelemetry.addData("Goal X (in)", goalPose.getX(DistanceUnit.INCH));
            multiTelemetry.addData("Goal Y (in)", goalPose.getY(DistanceUnit.INCH));
            multiTelemetry.addData("Distance to Goal (in)", getDistanceFromGoal());
        }

        multiTelemetry.update();
    }

}