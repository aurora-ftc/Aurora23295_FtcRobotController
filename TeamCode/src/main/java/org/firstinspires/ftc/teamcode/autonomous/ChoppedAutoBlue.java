package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.launchSubSystem.LaunchSystem;
@Autonomous(name = "ChoppedAutoBlue", group = "ChoppedAutos")
public class ChoppedAutoBlue extends LinearOpMode {
    private final MecanumDrive mecanumDrive = new MecanumDrive();
    private final LaunchSystem launchSystem = new LaunchSystem();
    private final double[] powerSteps = {0.68, 0.68};

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap, telemetry);
        launchSystem.init(0.10, 0.24, powerSteps, hardwareMap, telemetry);

        waitForStart();

        if(opModeIsActive()) {
            mecanumDrive.drive(1, 0 , 0, 0.4,telemetry); //Move Forward
            sleep(550);
            mecanumDrive.drive(0, 0 , -0.2, 1,telemetry); //Rotate Counterclockwise
            sleep(500);
            mecanumDrive.drive(0, 0 , 0, 0,telemetry); //Stop
            launchSystem.toggleLauncher(); //Launcher on
            launchSystem.updateLauncher();
            sleep(5000);
            launchSystem.liftUp();//Hit it up
            sleep(100);
            launchSystem.liftDown(); //Retract hitter
            sleep(200);
            launchSystem.toggleIntake(); //Intake on: Pushes queued balls
            sleep(2000);
            launchSystem.toggleIntake(); //Intake off
            sleep(2000);
            launchSystem.liftUp(); //Hit it up
            sleep(100);
            launchSystem.liftDown(); //Retract hitter
            sleep(200);
            launchSystem.toggleIntake(); //Intake on: Pushes queued balls
            sleep(2000);
            launchSystem.toggleIntake(); //Intake off
            sleep(2000);
            launchSystem.liftUp(); //Hit it up
            sleep(100);
            launchSystem.liftDown(); //Retract hitter
            sleep(200);
            launchSystem.toggleIntake(); //Intake on: Pushes queued balls
            sleep(2000);
            launchSystem.toggleIntake(); //Intake off
            launchSystem.toggleLauncher(); //Launcher off
            launchSystem.updateLauncher();
        }
    }

}
