package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.util.ServoGroup;

public class Turret {
    //Hardware
    private Servo tsf, tsb;

    //Hardware organization tool
    private ServoGroup turret;

    //Field
    private double pos;

    //State
    private enum State {
        MANUAL,
        AUTO;
    }
    private State currentState;

    //Adjustable
    private final double min = 0.1, max = 1.0, zero = 0.46; //Subject to change

    public void init(HardwareMap hwMap) {
        tsf = hwMap.get(Servo.class, "ts1");
        tsb = hwMap.get(Servo.class, "ts2");

        tsb.setDirection(Servo.Direction.FORWARD);
        tsf.setDirection(Servo.Direction.FORWARD);

        turret = new ServoGroup(tsf, tsb);

        turret.scaleRange(min, max);

        pos = zero;
        //tsb.setPosition(pos);
        turret.setPosition(pos);

        auto();
    }

    public void periodic(MecanumDrive drive) {
        if (currentState == State.AUTO)
            pos = clamp((diffTheta(drive) /
                    (1.27300194932 * Math.PI)) + zero); //Assumes 74-114 Gear ratio and 353rads
        else
            pos = zero;
        turret.setPosition(pos);
        //tsb.setPosition(pos);
    }

    public void clockwise() {
        if (currentState == State.AUTO)
            return;
        pos = clamp(pos - 0.02);
    }

    public void anticlockwise() {
        if (currentState == State.AUTO)
            return;
        pos = clamp(pos + 0.02);
    }

    public void manual() {
        currentState = State.MANUAL;
    }

    public void auto() {
        currentState = State.AUTO;
    }

    public void toggle() {
        if (currentState == State.MANUAL)
            auto();
        else
            manual();
    }

    public double diffTheta(MecanumDrive drive) {
        //Math diff in angle and stuff
        double thetaGoal = drive.angleToGoal();
        double currentTheta = drive.getOdoHeading(AngleUnit.RADIANS);
        double diffTheta = thetaGoal - currentTheta;

        //Normalize to -pi to pi
        diffTheta = Math.atan2(Math.sin(diffTheta), Math.cos(diffTheta));

        //Clamp to range -pi/2 to pi/2 for turret
        return Math.min(Math.PI/2, Math.max(-Math.PI/2, diffTheta));
    }

    public double clamp(double num) {
        return Math.max(min, Math.min(num, max));
    }

    public void log(Telemetry tele) {
        //They should be the same but may be slightly off, which would be good to know.
        tele.addData("Back Servo Pos", tsb.getPosition());
        tele.addData("Front Servo Pos", tsf.getPosition());
        tele.addData("State", currentState.name());
    }

}