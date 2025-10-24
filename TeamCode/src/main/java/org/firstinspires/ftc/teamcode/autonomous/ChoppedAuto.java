package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.launchSubSystem.LaunchSystem;

@Autonomous
public class ChoppedAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private LaunchSystem launchSystem = new LaunchSystem();
    private final double[] powerSteps = {0.68, 0.68};

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap, telemetry);
        launchSystem.init(0.10, 0.24, powerSteps, hardwareMap, telemetry);

        waitForStart();

        if(opModeIsActive()) {
            mecanumDrive.drive(1, 0 , 0, 0.4,telemetry);
            sleep(550);
            mecanumDrive.drive(0, 0 , 0.2, 1,telemetry);
            sleep(500);
            mecanumDrive.drive(0, 0 , 0.2, 0,telemetry);
            launchSystem.toggleLauncher();
            launchSystem.updateLauncher();
            sleep(5000);
            launchSystem.liftUp();
            sleep(100);
            launchSystem.liftDown();
            sleep(200);
            launchSystem.toggleIntake();
            sleep(2000);
            launchSystem.toggleIntake();
            sleep(2000);
            launchSystem.liftUp();
            sleep(100);
            launchSystem.liftDown();
            sleep(200);
            launchSystem.toggleIntake();
            sleep(2000);
            launchSystem.toggleIntake();
            sleep(2000);
            launchSystem.liftUp();
            sleep(100);
            launchSystem.liftDown();
            sleep(200);
            launchSystem.toggleIntake();
            sleep(2000);
            launchSystem.toggleIntake();
            launchSystem.toggleLauncher();
            launchSystem.updateLauncher();
        }
    }

}
