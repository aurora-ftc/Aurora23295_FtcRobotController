package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.DRIVE_KD;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.DRIVE_KP;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.HWMap;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.IS_FIELD_CENTRIC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.util.DcMotorGroup;
import org.firstinspires.ftc.teamcode.teleOp.util.PIDController;

import java.util.Locale;

public class MecanumDrive {

    public DcMotorGroup driveMotors;
    public boolean trackGoalOn = false;
    public Pose2D goalPose;
    GoBildaPinpointDriverRR odo;
    private GoBildaPinpointDriverRR odoRR;
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;
    private PIDController headingPID;
    private double newForward, newStrafe, theta;
    private String botPose;

    /**
     * smoothDrive: smooth out driving to ensure precise control
     *
     * @param input input value
     * @return smoothed value
     */
    public static double smoothDrive(double input) {
        return 0.3 * Math.tan(input * 1.2792);
    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        //Hardware Mapping
        frontLeftMotor = hwMap.get(DcMotorEx.class, HWMap.FL_MOTOR);
        frontRightMotor = hwMap.get(DcMotorEx.class, HWMap.FR_MOTOR);
        backLeftMotor = hwMap.get(DcMotorEx.class, HWMap.BL_MOTOR);
        backRightMotor = hwMap.get(DcMotorEx.class, HWMap.BR_MOTOR);

        odo = hwMap.get(GoBildaPinpointDriverRR.class, "odo");

        if (odo == null) {
            telemetry.addData("ERROR", "GoBildaPinpointDriverRR device 'odo' not found in hardware map!");
            telemetry.update();
            throw new IllegalStateException("GoBildaPinpointDriverRR device 'odo' not found in hardware map. Please check your robot configuration.");
        }

        imu = hwMap.get(IMU.class, HWMap.IMU);

        //Drive Motor Spin Directions
        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        driveMotors = new DcMotorGroup(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

        //Run W/out encoder
        driveMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Brake at 0 Power
        driveMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveMotors.setPower(0);

        //Current Bot Offsets: -41, 0, MM (11/13)
        //old bot odo.setOffsets(-41, 0, DistanceUnit.MM);
        odo.setOffsets(-13, -18.5, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriverRR.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //Current Bot Directions: FORWARD, REVERSED (11/13)
        odo.setEncoderDirections(GoBildaPinpointDriverRR.EncoderDirection.FORWARD,
                GoBildaPinpointDriverRR.EncoderDirection.REVERSED);

        //Calibrate ODO
        odo.resetPosAndIMU();

        headingPID = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD); // tune these values
        headingPID.setTarget(Math.PI / 2.0); //default goalPose heading = 0 degrees
        headingPID.previousTime = System.nanoTime() / 1e9;

        String PIDData = String.format(Locale.US, "{KP: %.3f, KI: %.3f, KD: %.3f}",
                DRIVE_KP, DRIVE_KI, DRIVE_KD);

        telemetry.addData("Status", "Initialized");
        if (odo != null) {
            telemetry.addData("X offset (Inches)", odo.getXOffset(DistanceUnit.INCH));
            telemetry.addData("Y offset (Inches)", odo.getYOffset(DistanceUnit.INCH));
            telemetry.addData("Odo Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Odo Heading Scalar", odo.getYawScalar());
        } else {
            telemetry.addData("X offset (Inches)", "N/A");
            telemetry.addData("Y offset (Inches)", "N/A");
            telemetry.addData("Odo Device Version Number:", "N/A");
            telemetry.addData("Odo Heading Scalar", "N/A");
        }
        telemetry.addData("PID Settings", PIDData);
        telemetry.update();

    }

    /**
     * drive: the primary drive method. Controls all drive motors together
     *
     * @param forward the target forward/reverse movement
     * @param strafe  the target left/right movement
     * @param rotate  the target turning movement
     * @param slow    speed modifier
     */
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

    /**
     * driveFieldOriented: drive modified to be in a headless mode. Forward is always the same direction, regardless of rotation
     *
     * @param forward the target forward/reverse movement
     * @param strafe  the target left/right movement
     * @param rotate  the target turning movement
     * @param slow    speed modifier
     */
    public void driveFieldOriented(double forward, double strafe, double rotate, double slow) {

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
        if (odo != null) {
            Pose2D pos = odo.getPosition();
            botPose = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                    pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH),
                    pos.getHeading(AngleUnit.DEGREES));
        } else {
            botPose = "{X: 0.000, Y: 0.000, H: 0.000}";
        }

        if (IS_FIELD_CENTRIC)
            drive(newForward, newStrafe, rotate, slow);
        else
            drive(forward, strafe, rotate, slow);
    }

    /**
     * headingPID: rotational PID controller. Locks rotation unless otherwise commanded.
     *
     * @param targetHeading the target heading
     * @return rotation movement
     */
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

    /**
     * debugTelemetry: complex telemetry for debugging drive methods
     *
     * @param telemetry telemetry object
     * @param slow      speed modifier. (no idea why this is here)
     */
    public void debugTelemetry(Telemetry telemetry, double slow) {
        TelemetryPacket packet = new TelemetryPacket();
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        multiTelemetry.addData("Speed Modifier", slow);
        multiTelemetry.addData("New Forward", newForward);
        multiTelemetry.addData("New Strafe", newStrafe);
        multiTelemetry.addData("Theta (Radians)", theta);
        if (odo != null) {
            multiTelemetry.addData("Odo Status", odo.getDeviceStatus());
            multiTelemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints the current refresh rate of the Pinpoint
            multiTelemetry.addLine();
            multiTelemetry.addData("Position", botPose);
            multiTelemetry.addLine();
            multiTelemetry.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));
        } else {
            multiTelemetry.addData("Odo Status", "NOT INITIALIZED");
            multiTelemetry.addData("Position", "N/A");
            multiTelemetry.addData("Heading (deg)", "N/A");
        }
        multiTelemetry.addLine();

        Canvas field = packet.fieldOverlay();
        drawRobot(field);
    }

    public void setPIDTargetHeading(double targetHeading) {
        headingPID.setTarget(targetHeading);
    }

    /**
     * resetOdoHeading: reset the heading of odometry
     * resetOdoPosition: reset the position of odometry
     */
    public void resetOdoHeading() {
        if (odo != null) {
            //Resets Heading and Position -STAY STILL FOR AT LEAST 0.25 SECONDS WHILE DOING SO FOR ACCURACY-
            odo.resetYaw();
            odo.update(GoBildaPinpointDriverRR.ReadData.ONLY_UPDATE_HEADING);
        }
    }

    public void resetOdoPosition() {
        if (odo != null) {
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,
                    AngleUnit.RADIANS, 0));
            updateOdo();
        }
    }

    /**
     * updateOdo: update the odometry pods with null safety catch<p>
     * updateOdoHeading: update the odometry heading </p>
     */
    public void updateOdo() {
        if (odo != null) {
            odo.update();
        }
    }

    public void updateOdoHeading() {
        if (odo != null) {
            odo.update(GoBildaPinpointDriverRR.ReadData.ONLY_UPDATE_HEADING);
        }
    }

    /**
     * getOdoHeading: get the odometry heading
     *
     * @return odo velocity
     */
    public double getOdoHeading(AngleUnit angleUnit) {
        if (odo != null) {
            odo.update(GoBildaPinpointDriverRR.ReadData.ONLY_UPDATE_HEADING);
            return odo.getPosition().getHeading(angleUnit);
        }
        return 0.0;
    }

    /**
     * getOdoVelocity: get the odometry velocity
     *
     * @return odo velocity
     */
    public double getOdoVelocity() {
        if (odo != null) {
            double velocity = Math.atan2(odo.getVelY(DistanceUnit.INCH),
                    odo.getVelX(DistanceUnit.INCH)) +
                    odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            return velocity;
        }
        return 0.0;
    }

    /**
     * getOdoPosition: get the position of the odometry
     *
     * @return position
     */
    public Pose2D getOdoPosition() {
        if (odo != null) {
            odo.update();
            return odo.getPosition();
        }
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    }

    public void setOdoPosition(Pose2D pose) {
        if (odo != null) {
            odo.setPosition(pose);
        }
    }

    /**
     * getOdoX: get the x value of the odometry
     *
     * @param distanceUnit the unit of distance to measure in
     * @return x position
     */
    public double getOdoX(DistanceUnit distanceUnit) {
        if (odo != null) {
            odo.update();
            return odo.getPosition().getX(distanceUnit);
        }
        return 0.0;
    }

    /**
     * getOdoY: get the y value of the odometry
     *
     * @param distanceUnit the unit of distance to measure in
     * @return y position
     */
    public double getOdoY(DistanceUnit distanceUnit) {
        if (odo != null) {
            odo.update();
            return odo.getPosition().getY(distanceUnit);
        }
        return 0.0;
    }

    /**
     * initTracker: initialize the goal tracker
     *
     * @param goalPose    the position of the goal
     * @param trackGoalOn enable/disable goal tracking
     */
    public void initTracker(Pose2D goalPose, boolean trackGoalOn) {
        this.goalPose = goalPose;
        this.trackGoalOn = trackGoalOn;
    }

    /**
     * <p>drawRobot: draw the robot to the FTC dashboard</p>
     *
     * @param field field map to use
     */
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

    /**
     * trackGoal: tracks the goal while moving on the field.
     *
     * @param forward target forward/reverse movement
     * @param strafe  target left/right movement
     * @param slow    speed modifier
     */
    public void trackGoal(double forward, double strafe, double slow) {
        updateOdo();

        double x = this.getOdoX(DistanceUnit.INCH);
        double y = this.getOdoY(DistanceUnit.INCH);

        double deltaY = goalPose.getY(DistanceUnit.INCH) - y;
        double deltaX = goalPose.getX(DistanceUnit.INCH) - x;

        double thetaGoal = AngleUnit.normalizeRadians(Math.atan2(deltaY, deltaX));


        double output = headingPID(thetaGoal);

        driveFieldOriented(forward, strafe, output, slow);
    }

    /**
     * getDistanceFromGoal: get the distance from the goal. Used in auto power calculations
     *
     * @return distance from goal
     */
    public double getDistanceFromGoal() {
        updateOdo();

        double x = getOdoX(DistanceUnit.INCH);
        double y = getOdoY(DistanceUnit.INCH);

        double deltaY = goalPose.getY(DistanceUnit.INCH) - y;
        double deltaX = goalPose.getX(DistanceUnit.INCH) - x;

        double distance = Math.hypot(deltaY, deltaX);

        return distance;
    }

    /**
     * toggleTrackGoal: toggles goal tracker
     * deactivateTrackGoal: deactivates goal tracker (used on init)
     */
    public void toggleTrackGoal() {
        trackGoalOn = !trackGoalOn;
    }

    public void deactivateTrackGoal() {
        trackGoalOn = false;
    }

    /**
     * updateTelemetry: standard telemetry update method.
     *
     * @param telemetry telemetry object
     * @param slow      speed modifier (no idea why its here again)
     */
    public void updateTelemetry(Telemetry telemetry, double slow) {

        TelemetryPacket packet = new TelemetryPacket();
        MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        updateOdo();

        multiTelemetry.addLine("--<== Mecanum Drive Telemetry ==>--");
        if (odo != null) {
            Pose2D pos = odo.getPosition();
            double headingDeg = getOdoHeading(AngleUnit.DEGREES);
            multiTelemetry.addData("OdoStatus", odo.getDeviceStatus());
            multiTelemetry.addData("Pinpoint Frequency", odo.getFrequency());
            multiTelemetry.addData("X (in)", pos.getX(DistanceUnit.INCH));
            multiTelemetry.addData("Y (in)", pos.getY(DistanceUnit.INCH));
            multiTelemetry.addData("Heading (deg)", headingDeg);
        } else {
            multiTelemetry.addData("OdoStatus", "NOT INITIALIZED");
            multiTelemetry.addData("Pinpoint Frequency", "N/A");
            multiTelemetry.addData("X (in)", "N/A");
            multiTelemetry.addData("Y (in)", "N/A");
            multiTelemetry.addData("Heading (deg)", "N/A");
        }

        Canvas field = packet.fieldOverlay();

        drawRobot(field);

        multiTelemetry.addData("Speed Modifier",
                slow == 0.35 ? "Slow (0.35)" : String.format(Locale.US, "Normal (%.2f)", slow));

        multiTelemetry.addData("Forward (calc)", newForward);
        multiTelemetry.addData("Strafe (calc)", newStrafe);
        multiTelemetry.addData("Theta (rad)", theta);

        if (trackGoalOn && goalPose != null) {
            multiTelemetry.addLine("--- Goal Tracking ---");
            double x = getOdoX(DistanceUnit.INCH);
            double y = getOdoY(DistanceUnit.INCH);
            double deltaY = goalPose.getY(DistanceUnit.INCH) - y;
            double deltaX = goalPose.getX(DistanceUnit.INCH) - x;
            double thetaGoal = AngleUnit.normalizeRadians(Math.atan2(deltaY, deltaX));

            multiTelemetry.addData("Goal X (in)", goalPose.getX(DistanceUnit.INCH));
            multiTelemetry.addData("Goal Y (in)", goalPose.getY(DistanceUnit.INCH));
            multiTelemetry.addData("Distance to Goal (in)", getDistanceFromGoal());

            if (DEBUG) {
                multiTelemetry.addData("deltaY", deltaY);
                multiTelemetry.addData("deltaX", deltaX);
                multiTelemetry.addData("Current X", x);
                multiTelemetry.addData("Current Y", y);
                multiTelemetry.addData("thetaGoal", thetaGoal);
            }
        }

        if (DEBUG) {
            multiTelemetry.addLine("--- Debug Info ---");
            multiTelemetry.addData("Bot Pose String", botPose);
        }

        multiTelemetry.update();
    }

}