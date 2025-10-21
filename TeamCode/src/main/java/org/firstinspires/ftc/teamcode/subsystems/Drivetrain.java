package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.GoBildaPinpointDriver;

public class Drivetrain {
    private DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    static public GoBildaPinpointDriver odo;

    public Drivetrain(HardwareMap hwMap) {
        leftFrontMotor  = hwMap.get(DcMotorEx.class, "leftFrontMotor");
        leftBackMotor  = hwMap.get(DcMotorEx.class, "leftBackMotor");
        rightFrontMotor = hwMap.get(DcMotorEx.class, "rightFrontMotor");
        rightBackMotor = hwMap.get(DcMotorEx.class, "rightBackMotor");

        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-0.4, 3.4, DistanceUnit.INCH);


    }

    public void drive(double leftFrontSpeed, double leftBackSpeed, double rightFrontSpeed, double rightBackSpeed) {
        leftFrontMotor.setPower(leftFrontSpeed);
        leftBackMotor.setPower(leftBackSpeed);
        rightFrontMotor.setPower(rightFrontSpeed);
        rightBackMotor.setPower(rightBackSpeed);
    }
}
