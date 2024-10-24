package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d beginPose = new Pose2d(28, -65.5, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(49, -38), Math.toRadians(0))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-48, -63, Math.toRadians(180)), Math.toRadians(180 + 45/2))
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(49 + 10.5, -38, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-48, -63, Math.toRadians(180)), Math.toRadians(180 + 45/2))
                        .setTangent(Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(60, -24, Math.toRadians(0)), Math.toRadians(90))
                        .setTangent(Math.toRadians(270 -45))
                        .splineToLinearHeading(new Pose2d(-48, -63, Math.toRadians(180)), Math.toRadians(180 + 45/2))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(-24, 9, Math.toRadians(0)), Math.toRadians(0))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}