package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;
import static org.firstinspires.ftc.teamcode.teleOp.util.Colors.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.util.Colors;
import org.firstinspires.ftc.teamcode.teleOp.util.MathUtils;

public class Indexer {
    protected enum State {
        INTAKE,
        OUTTAKE
    }

    protected enum Status {
        SORTING,
        IDLE;
    }

    //Sorting System
    private int[] intakePositions = {POSITIONS[1], POSITIONS[3], POSITIONS[5]};
    private int[] outtakePositions = {POSITIONS[0], POSITIONS[2], POSITIONS[4]};
    private int currentPositionIndex = 0;
    private int target = 0;
    private State currentState = State.INTAKE;
    private Status currentStatus = Status.IDLE;
    private int zero = 0;
    private boolean newBall = true;
    private boolean empty = false;
    private ElapsedTime emptyTimer = new ElapsedTime();

    //Hardware
    private Servo pushServo, gateServo;
    private CRServo rotaryServo;
    private AnalogInput elcAnalog;
    private DcMotorEx elcDigital;
    private RevColorSensorV3 cBottom, cLeft, cRight;
    private Servo indicator;

    //Classes
    private PIDController controller;

    public static boolean isEmpty(@NonNull RevColorSensorV3 cs) {
        if (cs == null) return true;

        double dist = cs.getDistance(DistanceUnit.INCH);
        Colors color = getColor(cs);

        if (color == UNKNOWN && dist >= 1.0)
            return true;

        return false;
    }

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

        //zero = (int) ((elcAnalog.getVoltage() / 3.3) * 4000);
        zero = 0;

        elcDigital.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elcDigital.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elcDigital.setDirection(DcMotor.Direction.FORWARD);

        rotaryServo.setDirection(DcMotorSimple.Direction.FORWARD);

        emptyTimer.reset();

        determineState();
    }

    public void periodic() {
        gatekeeper();
        determineStatus();

        if (!isEmpty(cBottom) && currentState == State.INTAKE && emptyTimer.seconds() >= 0.5) {
            if (isEmpty(cRight))
                moveUp();
            else if (isEmpty(cLeft))
                moveDown();
            else
                currentState = State.OUTTAKE;
            emptyTimer.reset();
        }

        if (isEmpty(cBottom))
            indicator.setPosition(0.7);
        else
            indicator.setPosition(0.4);

        //idk bout this zero thing rn
        double output = controller.calculate(elcDigital.getCurrentPosition() - zero, target);
        rotaryServo.setPower(Range.clip(MathUtils.safeSqrt(output), -1, 1));
        //rotaryServo.setPower(Range.clip(output, -1, 1));
    }

    public void detectNewBall() {
        if (emptyTimer.seconds() >= 0.2)
            empty = isEmpty(cBottom);
        if (isEmpty(cBottom) != empty) {
            emptyTimer.reset();
            newBall = true;
        } else {
            newBall = false;
        }
    }

    public void determineStatus() {
        double velocity = elcDigital.getVelocity();
        if (Math.abs(velocity) <= VELOCITY_THRESHOLD)
            currentStatus = Status.IDLE;
        else
            currentStatus = Status.SORTING;
    }

    public void determineState() {
        if(isEmpty(cBottom) && isEmpty(cLeft) && isEmpty(cRight))
            currentState = State.OUTTAKE;
        else
            currentState = State.INTAKE;
    }

    public void toggleState() {
        if (currentState == State.OUTTAKE)
            currentState = State.INTAKE;
        else
            currentState = State.OUTTAKE;
    }

    public void moveUp() {
        currentPositionIndex = (currentPositionIndex + 1) % 3;
        if (currentState == State.OUTTAKE)
            target = outtakePositions[currentPositionIndex];
        else
            target = intakePositions[currentPositionIndex];
    }

    public void moveDown() {
        currentPositionIndex = (currentPositionIndex + 2) % 3;
        if (currentState == State.OUTTAKE)
            target = outtakePositions[currentPositionIndex];
        else
            target = intakePositions[currentPositionIndex];
    }

    public void gatekeeper() {
        if (currentState == State.INTAKE)
            gateServo.setPosition(GATE_OPEN);
        else
            gateServo.setPosition(GATE_CLOSE);
    }

    public void log(Telemetry telemetry) {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tele.addData("State", currentState.toString());
        tele.addData("Position Index", currentPositionIndex);
        tele.addData("Target", target);
        tele.addData("Zero", zero);
        tele.addData("GatePos", gateServo.getPosition());
        tele.addData("Indexer Servo Power", rotaryServo.getPower());
        tele.addData("Empty Timer", emptyTimer.seconds());
        tele.addData("Status", currentStatus.toString());
        tele.addData("Rotary Velocity", elcDigital.getVelocity());

        tele.addLine();

        tele.addData("Bottom Color", getColor(cBottom).toString());
        tele.addData("Right Color", getColor(cRight).toString());
        tele.addData("Left Color", getColor(cLeft).toString());

        tele.addData("Bottom Color", isEmpty(cBottom));
        tele.addData("Right Color", isEmpty(cRight));
        tele.addData("Left Color", isEmpty(cLeft));
    }
}