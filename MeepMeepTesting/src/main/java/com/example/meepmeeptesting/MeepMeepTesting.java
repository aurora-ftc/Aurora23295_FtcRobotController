package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        TrajectoryVelocityConstraint velConstraint1 = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(5),
                new AngularVelocityConstraint(1)));

        TrajectoryAccelerationConstraint accelConstraint1 = new ProfileAccelerationConstraint(5);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(180)))
                        .forward(7)
                        .turn(Math.toRadians(-20.8))
                        .waitSeconds(8)
                        .splineTo(new Vector2d(36, 26),Math.toRadians(200))
                        .turn(Math.toRadians(70))
                        .waitSeconds(1)
                        .back(20, velConstraint1, accelConstraint1)
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineTo(new Vector2d(55, 13), Math.toRadians(90))
                        .turn(Math.toRadians(70))
                        .build());

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\Family\\Downloads\\dcField4.png\\")); }
        catch(IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}