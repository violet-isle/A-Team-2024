

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="FinalAuto24")
public class FinalAuto24 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime actionTime = new ElapsedTime();

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private AnalogInput AE = null;
    private RevTouchSensor TS;

    public class Lift {
        private DcMotorEx VS1;
        private DcMotorEx VS2;

        public Lift(HardwareMap hardwareMap) {
            VS1 = hardwareMap.get(DcMotorEx.class, "VS1");
            VS1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VS1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //VS1.setDirection(DcMotorSimple.Direction.REVERSE);

            VS2 = hardwareMap.get(DcMotorEx.class, "VS2");
            VS2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VS2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //VS2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public int getEncoderValue() {
            return VS1.getCurrentPosition(); // Replace with your encoder's method
        }
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VS1.setPower(0.8);
                    VS2.setPower(0.8);
                    initialized = true;
                }


                double pos = VS1.getCurrentPosition();
                packet.put("liftPos", pos);
                telemetry.addData("VS", pos);
                telemetry.update();
                if (pos > -58500.) {
                    return true;
                } else {
                    VS1.setPower(0.05);
                    VS2.setPower(0.05);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
        public class LiftSmallishUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VS1.setPower(0.8);
                    VS2.setPower(0.8);
                    initialized = true;
                }

                double pos = VS1.getCurrentPosition();
                packet.put("liftPos", pos);
                telemetry.addData("VS", pos);
                telemetry.update();
                if (pos > -33000.) {
                    return true;
                } else {
                    VS1.setPower(0.05);
                    VS2.setPower(0.05);
                    return false;
                }
            }
        }
        public Action liftSmallishUp() {
            return new LiftSmallishUp();
        }

         public class LiftSmallUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VS1.setPower(0.8);
                    VS2.setPower(0.8);
                    initialized = true;
                }

                double pos = VS1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > -13000) {
                    return true;
                } else {
                    VS1.setPower(0);
                    VS2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftSmallUp() {
            return new LiftSmallUp();
        }
        public class LiftMoreUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VS1.setPower(0.8);
                    VS2.setPower(0.8);
                    initialized = true;
                }

                double pos = VS1.getCurrentPosition();
                packet.put("liftPos", pos);
                telemetry.addData("VS", pos);
                telemetry.update();
                if (pos > -62000.) {
                    return true;
                } else {
                    VS1.setPower(0.05);
                    VS2.setPower(0.05);
                    return false;
                }
            }
        }
        public Action liftMoreUp() {
            return new LiftMoreUp();
        }
        
        public class LiftMoreDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VS1.setPower(-0.8);
                    VS2.setPower(-0.8);
                    initialized = true;
                }

                boolean touch = TS.isPressed();
                packet.put("touch", touch);
                if (!touch) {
                    return true;
                } else {
                    VS1.setPower(0);
                    VS2.setPower(0);
                    VS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    VS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    return false;
                }
            }
        }
        public Action liftMoreDown() {
            return new LiftMoreDown();
        }
        public class LiftReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                VS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                VS1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                return false;
            }
        }
        public Action liftReset() {
            return new LiftReset();
        }
    }

    private DcMotor LDW = null;
    private DcMotor RDW = null;
    private DcMotor BDW = null;


    public class Gripper {
        private Servo GR;

        public Gripper(HardwareMap hardwareMap) {
            GR = hardwareMap.get(Servo.class, "GR");
        }
        public class CloseGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR.setPosition(0);
                return false;
            }
        }
        public Action closeGripper() {
            return new CloseGripper();
        }

        public class OpenGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR.setPosition(0.27);
                return false;
            }
        }
        public Action openGripper() {
            return new OpenGripper();
        }
    }

    public class Wrist {
        private Servo WR;

        public Wrist(HardwareMap hardwareMap) {
            WR = hardwareMap.get(Servo.class, "WR");
        }
        public class WristIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                WR.setPosition(0);
                return false;
            }
        }
        public Action wristIn() {
            return new WristIn();
        }

        public class WristOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                WR.setPosition(0.5);
                return false;
            }
        }
        public Action wristOut() {
            return new WristOut();
        }
    }

    public class GripperFlipper {
        private Servo GF;

        public GripperFlipper(HardwareMap hardwareMap) {
            GF = hardwareMap.get(Servo.class, "UW");
        }
        public class FlipDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GF.setPosition(0);
                return false;
            }
        }
        public Action flipDown() {
            return new FlipDown();
        }

        public class FlipUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GF.setPosition(0.055);
                return false;
            }
        }
        public Action flipUp() {
            return new FlipUp();
        }
    }


    public class Horizontal {
        private CRServo HS;

        public Horizontal(HardwareMap hardwareMap) {
            HS = hardwareMap.get(CRServo.class, "HS");
        }
        public class SlidePunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                HS.setPower(1);
                return false;

            }
        }

        public Action slidePunch() {
            return new SlidePunch();
        }
        public class SlidePull implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                HS.setPower(-1);
                return false;

            }
        }
        public Action slidePull() {
            return new SlidePull();
        }
        public class SlideStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                HS.setPower(0);
                return false;

            }
        }
        public Action slideStop() {
            return new SlideStop();
        }
    }

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftBack = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack = hardwareMap.get(DcMotor.class, "BR");
        AE = hardwareMap.get(AnalogInput.class, "AE");


        LDW = hardwareMap.get(DcMotor.class, "FL");
        RDW = hardwareMap.get(DcMotor.class, "BR");
        BDW = hardwareMap.get(DcMotor.class, "FR");
        TS = hardwareMap.get(RevTouchSensor.class, "TS");

        Pose2d beginPose = new Pose2d(25, -66, Math.toRadians(90));



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Pose2d beginPose = new Pose2d(8, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Gripper GR = new Gripper(hardwareMap);

        GripperFlipper GF = new GripperFlipper(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        Horizontal HS = new Horizontal(hardwareMap);


        Wrist WR = new Wrist(hardwareMap);

        Actions.runBlocking(lift.liftReset());


        Actions.runBlocking(GF.flipUp());

        Actions.runBlocking(GR.closeGripper());

       Action chamber = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(0, -35), Math.toRadians(90))
                .build();

       Action back = drive.actionBuilder(new Pose2d(0, -35, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(0, -48), Math.toRadians(90))
                .build();



        Action block1 = drive.actionBuilder(new Pose2d(0, -48, Math.toRadians(90)))
                .setTangent(Math.toRadians(22.5))
                .splineToLinearHeading(new Pose2d(40, -40, Math.toRadians(270)), Math.toRadians(22.5))
                .setTangent(Math.toRadians(90-12.5))
                .splineToConstantHeading(new Vector2d(48, -12), Math.toRadians(270+45))
                .splineToConstantHeading(new Vector2d(50, -55), Math.toRadians(90+45))
                .setTangent(Math.toRadians(90+45))
                .splineToConstantHeading(new Vector2d(58, -12), Math.toRadians(270+45))
                .splineToConstantHeading(new Vector2d(64, -24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(64, -55), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -50), Math.toRadians(270))
                .build();

        Action grab1 = drive.actionBuilder(new Pose2d(48, -50, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(48, -58), Math.toRadians(-90))
                .build();
        Action chamber2 = drive.actionBuilder(new Pose2d(48, -58, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(9, -34.5, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action back1 = drive.actionBuilder(new Pose2d(9, -34.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90)).setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(-90)), Math.toRadians(90+45))
                .build();

        Action grab2 = drive.actionBuilder(new Pose2d(48, -50, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(48, -56), Math.toRadians(-90))
                .build();
        Action chamber3 = drive.actionBuilder(new Pose2d(48, -56, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90+45))
                .splineToLinearHeading(new Pose2d(-12, -34.5, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action back2 = drive.actionBuilder(new Pose2d(-12, -34.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90)).setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(-90)), Math.toRadians(90+45))
                .build();

        Action grab3 = drive.actionBuilder(new Pose2d(48, -50, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(48, -56), Math.toRadians(-90))
                .build();
        Action chamber4 = drive.actionBuilder(new Pose2d(48, -56, Math.toRadians(-90)))
                .setTangent(Math.toRadians(90+45))
                .splineToLinearHeading(new Pose2d(12, -34.5, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action back3 = drive.actionBuilder(new Pose2d(12, -34.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90)).setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(-90)), Math.toRadians(90+45))
                .build();


        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "starting");
        telemetry.update();



        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        lift.liftUp()
                                ),
                                chamber,
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        HS.slidePunch(),
                                        GF.flipUp(),
                                        new SleepAction(0.5),
                                        HS.slideStop()
                                )
                        ),
                        GR.openGripper(),
                        GF.flipDown(),
                        back,
                        new ParallelAction(
                            lift.liftMoreDown(),
                            block1
                        ),
                        lift.liftSmallUp(),
                        grab1,
                        GR.closeGripper(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        lift.liftMoreUp()
                                ),
                                chamber2,
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        HS.slidePunch(),
                                        GF.flipUp(),
                                        new SleepAction(0.5),
                                        HS.slideStop()
                                )
                        ),
                        GR.openGripper(),
                        GF.flipDown(),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(2),
                                        lift.liftMoreDown(),
                                        lift.liftSmallUp()
                                ),
                                back1
                        ),
                        grab2,
                        GR.closeGripper(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        lift.liftMoreUp()
                                ),
                                chamber3,
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        HS.slidePunch(),
                                        GF.flipUp(),
                                        new SleepAction(0.5),
                                        HS.slideStop()
                                )
                        ),
                        GR.openGripper(),
                        new SleepAction(0.5),
                        GF.flipDown(),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(2),
                                        lift.liftMoreDown(),
                                        lift.liftSmallUp()
                                ),
                                back2
                        ),
                        grab3,
                        GR.closeGripper(),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.1),
                                        lift.liftMoreUp()
                                ),
                                chamber4,
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        HS.slidePunch(),
                                        GF.flipUp(),
                                        new SleepAction(0.5),
                                        HS.slideStop()
                                )
                        )

                )
        );


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("VS1", lift.getEncoderValue());
            telemetry.addData("Back Deadwheel", BDW.getCurrentPosition());
            telemetry.addData("Right Deadwheel", RDW.getCurrentPosition());
            telemetry.addData("left Deadwheel", LDW.getCurrentPosition());

            telemetry.update();
        }
    }

}
