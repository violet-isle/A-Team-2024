package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d beginPose = new Pose2d(25, -66, Math.toRadians(90));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, -40, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)

                        .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, -48), Math.toRadians(90))
                        .setTangent(Math.toRadians(22.5))
                        .splineToLinearHeading(new Pose2d(40, -40, Math.toRadians(270)), Math.toRadians(22.5))
                        .setTangent(Math.toRadians(90-12.5))
                        .splineToConstantHeading(new Vector2d(48, -12), Math.toRadians(270+45))
                        .splineToConstantHeading(new Vector2d(50, -57), Math.toRadians(90+45))
                        .setTangent(Math.toRadians(90+45))
                        .splineToConstantHeading(new Vector2d(58, -12), Math.toRadians(270+45))
                        .splineToConstantHeading(new Vector2d(64, -24), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(64, -57), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(48, -50), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(48, -62), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(6, -40, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(6, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90)).setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(-90)), Math.toRadians(90+45))
                        .splineToConstantHeading(new Vector2d(48, -64), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90+45))
                        .splineToLinearHeading(new Pose2d(-12, -40, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-12, -34), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90)).setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(-90)), Math.toRadians(90+45))
                        .splineToConstantHeading(new Vector2d(48, -56), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90+45))
                        .splineToLinearHeading(new Pose2d(12, -34.5, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90)).setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(-90)), Math.toRadians(90+45))




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