package org.firstinspires.ftc.teamcode.teleOp.subSystems;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.*;
import static org.firstinspires.ftc.teamcode.teleOp.util.Colors.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleOp.util.Colors;

public class Indexer {
    //Sorting System
    protected enum State {
        INTAKE,
        OUTTAKE
    }
    protected enum Status {
        SORTING,
        IDLE;
    }
    private int[] outtakePositions = {POSITIONS[1], POSITIONS[3], POSITIONS[5]};
    private int[] intakePositions = {POSITIONS[0], POSITIONS[2], POSITIONS[4]};
    private int currentPositionIndex = 0;
    private int target = intakePositions[1];
    private State currentState = State.INTAKE;
    private Status currentStatus = Status.IDLE;
    private int zero;
    private double current;

    //Event Timers
    private ElapsedTime emptyTimer = new ElapsedTime();
    private ElapsedTime gateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    //Hardware
    private Servo pushServo, gateServo;
    private AnalogInput elcAnalog;
    private DcMotorEx elcDigital, indexerMotor;
    private RevColorSensorV3 cBottom, cLeft, cRight;
    private boolean needTurn = false, update = false;

    //Classes
    private PIDController controller;
    private Indicator light;

    public static boolean isEmpty(@NonNull RevColorSensorV3 cs) {
        if (cs == null) return true;

        double dist = cs.getDistance(DistanceUnit.INCH);
        Colors color = getColor(cs);

        if (color == UNKNOWN && dist >= 1.0)
            return true;

        return false;
    }

    public void init(HardwareMap map) {

        //Hardware mapping
        indexerMotor = map.get(DcMotorEx.class, HWMap.INDEXER_MOTOR);
        pushServo = map.get(Servo.class, HWMap.PUSH_SERVO);
        gateServo = map.get(Servo.class, HWMap.GATE_SERVO);

        elcAnalog = map.get(AnalogInput.class, HWMap.ELC_ANALOG);
        elcDigital = map.get(DcMotorEx.class, HWMap.ELC_DIGITAL);

        cBottom = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_BOTTOM);
        cLeft = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_LEFT);
        cRight = map.get(RevColorSensorV3.class, HWMap.COLOR_SENSOR_RIGHT);

        light = new Indicator();
        light.init(map);
        light.off();

        cBottom.setGain(revColorSensorGain);
        cRight.setGain(revColorSensorGain);
        cLeft.setGain(revColorSensorGain);

        controller = new PIDController(PID.Rotary.ROTARY_KP, PID.Rotary.ROTARY_KI, PID.Rotary.ROTARY_KD);
        controller.setPID(PID.Rotary.ROTARY_KP, PID.Rotary.ROTARY_KI, PID.Rotary.ROTARY_KD);

        pushServo.scaleRange(0.05, 0.25);
        gateServo.scaleRange(0.1, 0.25);

        pushServo.setPosition(PUSH_IDLE);
        gateServo.setPosition(GATE_CLOSE);

        zero = (int) ((elcAnalog.getVoltage() / 3.3) * 4000) - 510;

        elcDigital.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elcDigital.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elcDigital.setDirection(DcMotor.Direction.FORWARD);

        indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);

        emptyTimer.reset();
        gateTimer.reset();
        shootTimer.reset();

        determineState();
    }

    public void periodic(Intake intake) {

        gatekeeper();
        determineStatus();
        updateBlip(intake);

        controller.setPID(PID.Rotary.ROTARY_KP, PID.Rotary.ROTARY_KI, PID.Rotary.ROTARY_KD);

        if (shootTimer.seconds() >= 0.15)
            pushServo.setPosition(PUSH_IDLE);

        if (shootTimer.seconds() >= 0.25 && needTurn) {
            clockwise();
            needTurn = false;
        }

        if (!isEmpty(cBottom) && currentState == State.INTAKE
                && currentStatus == Status.IDLE && emptyTimer.seconds() >= 0.2) {
            if (isEmpty(cRight))
                counterClockwise();
            else if (isEmpty(cLeft))
                clockwise();
            else
                currentState = State.OUTTAKE;
            emptyTimer.reset();
        }

        current = elcDigital.getCurrentPosition() - zero;
        double output = controller.calculate(current, target);

        if (output < -0.01)
            output -= PID.Rotary.ROTARY_KS;
        else if (output > 0.01)
            output += PID.Rotary.ROTARY_KS;

        indexerMotor.setPower(Range.clip(output, -1, 1));

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("target", target);
        packet.put("current", current);
        packet.put("output", output * 1000);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }

    public void determineStatus() {
        double velocity = elcDigital.getVelocity();
        if (Math.abs(velocity) <= VELOCITY_THRESHOLD)
            currentStatus = Status.IDLE;
        else
            currentStatus = Status.SORTING;
    }

    public void shoot() {
        pushServo.setPosition(PUSH_PUSH);
        shootTimer.reset();
        needTurn = true;
    }
    public void determineState() {
        if(!isEmpty(cBottom) && !isEmpty(cLeft) && !isEmpty(cRight)) {
            currentState = State.OUTTAKE;
            update = true;
        } else
            currentState = State.INTAKE;
    }

    public void updateBlip(Intake intake) {
        if (update && currentState == State.OUTTAKE) {
            intake.outtakeBlip();
            update = false;
        }
    }

    public void toggleState() {
        if (currentState == State.OUTTAKE)
            currentState = State.INTAKE;
        else {
            currentState = State.OUTTAKE;
            update = true;
        }
    }

    public void clockwise() {
        gateTimer.reset();
        currentPositionIndex = (currentPositionIndex + 1) % 3;
        if (currentState == State.OUTTAKE)
            target = outtakePositions[currentPositionIndex];
        else
            target = intakePositions[currentPositionIndex];
    }

    public void counterClockwise() {
        gateTimer.reset();
        currentPositionIndex = (currentPositionIndex + 2) % 3;
        if (currentState == State.OUTTAKE)
            target = outtakePositions[currentPositionIndex];
        else
            target = intakePositions[currentPositionIndex];
    }

    public void gatekeeper() {
        if (gateTimer.seconds() <= 0.65) {
            gateServo.setPosition(GATE_CLOSE);
            return;
        }
        if (currentState == State.INTAKE && currentStatus == Status.IDLE)
            gateServo.setPosition(GATE_OPEN);
        else
            gateServo.setPosition(GATE_CLOSE);
    }

    public void lightkeeper() {
        if (currentState == State.INTAKE)
            light.green();
        else
            light.red();
    }

    public void log(Telemetry telemetry) {
        MultipleTelemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tele.addData("State", currentState.toString());
        tele.addData("Position Index", currentPositionIndex);
        tele.addData("Target", target);
        tele.addData("Zero", zero);
        tele.addData("GatePos", gateServo.getPosition());
        tele.addData("Indexer Servo Power", indexerMotor.getPower());
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