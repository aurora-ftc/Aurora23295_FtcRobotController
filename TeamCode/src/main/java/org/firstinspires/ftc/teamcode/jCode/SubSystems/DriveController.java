package org.firstinspires.ftc.teamcode.jCode.SubSystems;

import static org.firstinspires.ftc.teamcode.jCode.Configuration.DRIVE_KD;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.jCode.Configuration.DRIVE_KP;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.jCode.Configuration.HWMap;
import org.firstinspires.ftc.teamcode.teleOp.util.DcMotorGroup;

public class DriveController {

    private DcMotorGroup driveMotors;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriverRR odo;
    private PIDController pidController;

    public void init(HardwareMap map, Telemetry telemetry) {
        frontLeft = map.get(DcMotorEx.class, HWMap.flMotor);
        frontRight = map.get(DcMotorEx.class, HWMap.frMotor);
        backLeft = map.get(DcMotorEx.class, HWMap.blMotor);
        backRight = map.get(DcMotorEx.class, HWMap.brMotor);

        odo = map.get(GoBildaPinpointDriverRR.class, HWMap.odo);

        if (odo == null) {
            telemetry.addData("ERROR", "GoBildaPinpointDriverRR device 'odo' not found in hardware map!");
            telemetry.update();
            throw new IllegalStateException("GoBildaPinpointDriverRR device 'odo' not found in hardware map. Please check your robot configuration.");
        }

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        driveMotors = new DcMotorGroup(frontLeft, frontRight, backLeft, backRight);

        driveMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveMotors.setPower(0);

        odo.setOffsets(-13, -18.5, DistanceUnit.CM);
        odo.setEncoderResolution(GoBildaPinpointDriverRR.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriverRR.EncoderDirection.FORWARD, GoBildaPinpointDriverRR.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        pidController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        pidController.setSetPoint(0.0);
        pidController.calculate();
    }

    public void drive(double forward, double strafe, double rotation, double slow) {
        double frontLeftPower = forward + strafe - rotation;
        double backLeftPower = forward - strafe - rotation;
        double frontRightPower = forward - strafe + rotation;
        double backRightPower = forward + strafe + rotation;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeft.setPower(slow * maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(slow * maxSpeed * (frontRightPower / maxPower));
        backLeft.setPower(slow * maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(slow * maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldOriented(double forward, double strafe, double rotation, double slow) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        updateOdoHeading();

        double heading = getOdoHeading(AngleUnit.RADIANS);
        theta = AngleUnit.normalizeRadians(theta - heading);

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        drive(newForward, newStrafe, rotation, slow);
    }

    public double headingPID(double targetHeading) {
        pidController.setSetPoint(targetHeading);
        updateOdoHeading();

        double currentHeading = getOdoHeading(AngleUnit.RADIANS);

        double output = pidController.calculate();

        return -output;
    }

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

    public double getOdoHeading(AngleUnit angleUnit) {
        if (odo != null) {
            updateOdoHeading();
            return odo.getPosition().getHeading(angleUnit);
        }
        return 0.0;
    }

    public Pose2D getOdoPosition() {
        if (odo != null) {
            updateOdo();
            return odo.getPosition();
        }
        return null;
    }

    public void driveTelemetry(Telemetry telemetry) {
        telemetry.addLine("Mecanum Drive System");
        telemetry.addData("Driver Status", "OK");
        telemetry.addData("Odometry Status: ", odo.getDeviceStatus());
        telemetry.addData("Movement Values", "ERROR: No Values Found");
        telemetry.addLine();
    }
}
