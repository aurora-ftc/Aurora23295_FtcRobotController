package org.firstinspires.ftc.teamcode.roadrunner.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.LaunchIntakeSystem;

@Autonomous(name = "ChoppedAuto2Red", group = "ChoppedAutos2")
public class ChoppedAuto2Red extends LinearOpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private LaunchIntakeSystem launchIntakeSystem = new LaunchIntakeSystem();
    private final double[] powerSteps = {0.68, 0.68};

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive.init(hardwareMap, telemetry);
        launchIntakeSystem.init(powerSteps, hardwareMap);

        waitForStart();

        if(opModeIsActive()) {
            mecanumDrive.drive(1, 0 , 0, 0.4); //Move Forward
            sleep(550);
            mecanumDrive.drive(0, 0 , 0.2, 1); //Rotate Clockwise
            sleep(500);
            mecanumDrive.drive(0, 0 , 0, 0); //Stop
            launchIntakeSystem.toggleLauncher(); //Launcher on
            //launchIntakeSystem.updateLauncher();
            sleep(5000);
            launchIntakeSystem.liftUp();//Hit it up
            sleep(100);
            launchIntakeSystem.liftDown(); //Retract hitter
            sleep(200);
            launchIntakeSystem.toggleIntake(); //Intake on: Pushes queued balls
            sleep(2000);
            launchIntakeSystem.toggleIntake(); //Intake off
            sleep(2000);
            launchIntakeSystem.liftUp(); //Hit it up
            sleep(100);
            launchIntakeSystem.liftDown(); //Retract hitter
            sleep(200);
            launchIntakeSystem.toggleIntake(); //Intake on: Pushes queued balls
            sleep(2000);
            launchIntakeSystem.toggleIntake(); //Intake off
            sleep(2000);
            launchIntakeSystem.liftUp(); //Hit it up
            sleep(100);
            launchIntakeSystem.liftDown(); //Retract hitter
            sleep(200);
            launchIntakeSystem.toggleIntake(); //Intake on: Pushes queued balls
            sleep(2000);
            launchIntakeSystem.toggleIntake(); //Intake off
            launchIntakeSystem.toggleLauncher(); //Launcher off
            //launchIntakeSystem.updateLauncher();
            sleep(500);
            mecanumDrive.driveFieldOriented(0.7, 0 , -0.5, 1);//Rotate intake to balls
            sleep(1000);
            mecanumDrive.drive(0, 0 , 0, 0);
            sleep(300);
            mecanumDrive.drive(-0.3, 0, 0, 0);
            sleep(2000);
            mecanumDrive.drive(0, 0 , 0, 0);
            sleep(300);
        }
    }

}