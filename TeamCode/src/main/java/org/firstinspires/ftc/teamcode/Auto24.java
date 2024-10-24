

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Auto24")
public class Auto24 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotorEx VS1 = null;
    private DcMotorEx VS2 = null;

    private DcMotor LDW = null;
    private DcMotor RDW = null;
    private DcMotor BDW = null;

    private Servo GR;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftBack = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack = hardwareMap.get(DcMotor.class, "BR");

        VS1 = hardwareMap.get(DcMotorEx.class, "VS1");
        VS2 = hardwareMap.get(DcMotorEx.class, "VS2");

        LDW = hardwareMap.get(DcMotor.class, "FL");
        RDW = hardwareMap.get(DcMotor.class, "BR");
        BDW = hardwareMap.get(DcMotor.class, "FR");

        GR = hardwareMap.get(Servo.class, "GR");


        VS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VS2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //VS1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //VS2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Pose2d beginPose = new Pose2d(28, -65.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);






        waitForStart();
        runtime.reset();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
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
                        //.afterDisp(0, liftBlock())
                        .build());


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Back Deadwheel", BDW.getCurrentPosition());
            telemetry.addData("Right Deadwheel", RDW.getCurrentPosition());
            telemetry.addData("left Deadwheel", LDW.getCurrentPosition());
            telemetry.update();
        }
    }
    public Action liftBlock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                VS1.setTargetPosition(3200);
                VS2.setTargetPosition(3200);
                ((DcMotorEx) VS1).setVelocity(1000);
                ((DcMotorEx) VS2).setVelocity(1000);

                double vel = VS1.getVelocity();
                packet.put("liftVelocity", vel);
                return vel < 100.0;
            }
        };
    }
    public Action lowerBlock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                VS1.setTargetPosition(0);
                VS2.setTargetPosition(0);
                ((DcMotorEx) VS1).setVelocity(1000);
                ((DcMotorEx) VS2).setVelocity(1000);

                double vel = VS1.getVelocity();
                packet.put("liftVelocity", vel);
                return vel < 100.0;
            }
        };
    }
    public Action grabBlock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR.setPosition(1);
                return true;
            }
        };
    }
    public Action releaseBlock() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR.setPosition(0);
                return true;
            }
        };
    }

}
