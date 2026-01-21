package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;
import static org.firstinspires.ftc.teamcode.teleOp.util.Colors.*;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleOp.util.Colors;

public class Indexer {
    protected enum State {
        INTAKE,
        OUTTAKE
    }
    protected enum Control {
        MANUAL,
        AUTO
    }

    //Sorting System
    private Colors[] intakeDynamic, intakeStatic, outtakeDynamic, outtakeStatic;
    private int[] intakePositions = {POSITIONS[1], POSITIONS[3], POSITIONS[5]};
    private int[] outtakePositions = {POSITIONS[0], POSITIONS[2], POSITIONS[4]};
    private int currentPositionIndex = 0;
    private int target = 0;
    private State currentState = State.INTAKE;
    private Control currentControl = Control.MANUAL;

    //Hardware
    Servo pushServo, gateServo;
    private CRServo rotaryServo;
    private AnalogInput elcAnalog;
    private DcMotorEx elcDigital;
    private RevColorSensorV3 cBottom, cLeft, cRight;
    private Servo indicator;
    private byte currentPos = 0;

    //Classes
    private PIDController controller;

    public void init(HardwareMap map) {
        rotaryServo = map.get(CRServo.class, HWMap.ROTARY_SERVO);

        pushServo = map.get(Servo.class, HWMap.PUSH_SERVO);
        gateServo = map.get(Servo.class, HWMap.GATE_SERVO);

        elcAnalog = map.get(AnalogInput.class, HWMap.ELC_ANALOG);
        elcDigital = map.get(DcMotorEx.class, HWMap.ELC_DIGITAL);

        cBottom = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_BOTTOM);
        cLeft = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_LEFT);
        cRight = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_RIGHT);

        indicator = map.get(Servo.class, HWMap.LIGHT);

        cBottom.setGain(revColorSensorGain);
        cRight.setGain(revColorSensorGain);
        cLeft.setGain(revColorSensorGain);

        controller = new PIDController(ROTARY_KP, ROTARY_KI, ROTARY_KD);
        controller.setPID(ROTARY_KP, ROTARY_KI, ROTARY_KD);

        pushServo.scaleRange(0.05, 0.25);
        gateServo.scaleRange(0.1, 0.25); //guess

        pushServo.setPosition(PUSH_IDLE); //down
        gateServo.setPosition(GATE_CLOSE);

        elcDigital.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elcDigital.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elcDigital.setDirection(DcMotorSimple.Direction.FORWARD);

        rotaryServo.setDirection(DcMotorSimple.Direction.FORWARD);

        determineState();
        detectBalls();
    }

    public void turnCC(byte rotations) {
        currentPos = (byte) ((currentPos + rotations) % 6);
    }

    public void turnC(byte pose) {
        if (pose >= 0 && pose <6)
            currentPos = pose;
    }

    public void determineState() {
        if(getColor(cBottom) != UNKNOWN &&
           getColor(cLeft) != UNKNOWN &&
           getColor(cRight) != UNKNOWN)
        {
            currentState = State.OUTTAKE;
        } else {
            currentState = State.INTAKE;
        }
    }

    public void detectBalls() {
        if (currentState == State.INTAKE) {
            intakeStatic = new Colors[] {getColor(cBottom), getColor(cLeft), getColor(cRight)};
            intakeDynamic = new Colors[] {getColor(cBottom), getColor(cLeft), getColor(cRight)};
        }
    }

    public void wholeCounterclockwise(){
        if (currentState == State.INTAKE) {
            Colors i0 = intakeStatic[0];
            Colors i1 = intakeStatic[1];
            Colors i2 = intakeStatic[2];

            //rotation code

            intakeStatic = new Colors[] {i1, i2, i0};
        }
    }

    public void wholeClockwise(){
        if (currentState == State.INTAKE) {
            Colors i0 = intakeStatic[0];
            Colors i1 = intakeStatic[1];
            Colors i2 = intakeStatic[2];

            //rotation code

            intakeStatic = new Colors[] {i2, i0, i1};
        }
    }

    public void toggleControl() {
        if (currentControl == Control.MANUAL)
            currentControl = Control.AUTO;
        else
            currentControl = Control.MANUAL;
    }

    public void toggleState() {
        if (currentState == State.OUTTAKE)
            currentState = State.INTAKE;
        else
            currentState = State.OUTTAKE;
    }

    public void moveUp() {
        if (currentControl == Control.MANUAL) {
            currentPositionIndex = (currentPositionIndex + 1) % 3;
            if (currentState == State.OUTTAKE)
                target = outtakePositions[currentPositionIndex];
            else
                target = intakePositions[currentPositionIndex];
        }
    }

    public void moveDown() {
        if (currentControl == Control.MANUAL) {
            currentPositionIndex = (currentPositionIndex + 2) % 3;
            if (currentState == State.OUTTAKE)
                target = outtakePositions[currentPositionIndex];
            else
                target = intakePositions[currentPositionIndex];
        }
    }

    public void periodic() {
        double output = controller.calculate(elcDigital.getCurrentPosition(), target);

        int sign = 1;

        if (output < 0) {
            sign = -1;
            output *= -1;
        }

        output = Math.sqrt(output);
        output *= sign;

        if (currentControl == Control.AUTO) {
            if (currentState == State.INTAKE)
                determineState();
        }

        if (currentState == State.INTAKE)
            gateServo.setPosition(GATE_OPEN);
        else
            gateServo.setPosition(GATE_CLOSE);

        rotaryServo.setPower(Range.clip(output, -1, 1));
    }

    public void log(Telemetry tele) {
        tele.addData("State", currentState.toString());
        tele.addData("Control", currentControl.toString());
        tele.addData("Position Index", currentPositionIndex);
        tele.addData("Target", target);
        tele.addData("GatePos", gateServo.getPosition());
    }

    public void goToGreen() {}

    public void gateGoTo(double pos) {
        gateServo.setPosition(pos);
    }

}