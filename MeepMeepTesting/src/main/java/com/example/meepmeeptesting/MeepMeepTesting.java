package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d beginPose = new Pose2d(-39, -64.5, Math.toRadians(180));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(0, -48), Math.toRadians(90))
                .setTangent(Math.toRadians(22.5))
                .splineToLinearHeading(new Pose2d(40, -36, Math.toRadians(-90)), Math.toRadians(22.5))
                .setTangent(Math.toRadians(90-12.5))
                .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(270+45))
                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(270))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(270+45))

                .splineToConstantHeading(new Vector2d(64, -24), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(64, -60), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(64, -50), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(64, -67), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(9, -33, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(0, -48), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(64, -67), Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-9, -34.5, Math.toRadians(90)), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(65, -65), Math.toRadians(0))



                        /***.splineToConstantHeading(new Vector2d(55+3, -3), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(64+3, -24), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(64+3, -50), Math.toRadians(270))***/



                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}