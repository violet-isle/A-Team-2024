

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="SlapAllAuto24")
public class SlapAllAuto24 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private AnalogInput AE = null;

    public class Lift {
        private DcMotorEx VS1;
        private DcMotorEx VS2;

        public Lift(HardwareMap hardwareMap) {
            VS1 = hardwareMap.get(DcMotorEx.class, "VS1");
            VS1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VS1.setDirection(DcMotorSimple.Direction.FORWARD);

            VS2 = hardwareMap.get(DcMotorEx.class, "VS2");
            VS2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            VS2.setDirection(DcMotorSimple.Direction.FORWARD);
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
                if (pos > -2000.) {
                    return true;
                } else {
                    VS1.setPower(0);
                    VS2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    VS1.setPower(-0.8);
                    VS2.setPower(-0.8);
                    initialized = true;
                }

                double pos = VS1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < -1250) {
                    return true;
                } else {
                    VS1.setPower(0);
                    VS2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
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

                double pos = VS1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < -50) {
                    return true;
                } else {
                    VS1.setPower(0);
                    VS2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftMoreDown() {
            return new LiftMoreDown();
        }
    }


    private DcMotor LDW = null;
    private DcMotor RDW = null;
    private DcMotor BDW = null;




    public class SlapGripper {
        private Servo GR2;

        public SlapGripper(HardwareMap hardwareMap) {
            GR2 = hardwareMap.get(Servo.class, "GR2");
        }
        public class CloseGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR2.setPosition(0.5);
                return false;
            }
        }
        public Action closeGripper() {
            return new CloseGripper();
        }

        public class OpenGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR2.setPosition(1);
                return false;
            }
        }
        public Action openGripper() {
            return new OpenGripper();
        }
    }


    public class SlapWrist {
        private Servo WR2;

        public SlapWrist(HardwareMap hardwareMap) {
            WR2 = hardwareMap.get(Servo.class, "WR2");
        }
        public class ChamberFlip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                WR2.setPosition(0.11);
                return false;
            }
        }
        public Action chamberFlip() {
            return new ChamberFlip();
        }

        public class WallFlip implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                WR2.setPosition(1);
                return false;
            }
        }
        public Action wallFlip() {
            return new WallFlip();
        }
    }


    public class SlapArm {
        private Servo ARM;

        public SlapArm(HardwareMap hardwareMap) {
            ARM = hardwareMap.get(Servo.class, "ARM");
        }
        public class ChamberArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ARM.setPosition(0.13);
                return false;
            }
        }
        public Action chamberArm() {
            return new ChamberArm();
        }

        public class WallArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ARM.setPosition(0.028);
                return false;
            }
        }
        public Action wallArm() {
            return new WallArm();
        }
        public class WallArm1 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ARM.setPosition(0.05);
                return false;
            }
        }
        public Action wallArm1() {
            return new WallArm1();
        }


        public class InitArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ARM.setPosition(0.16);
                return false;
            }
        }
        public Action initArm() {
            return new InitArm();
        }

    }


    public class Horizontal {
        private CRServo HS;

        public Horizontal(HardwareMap hardwareMap) {
            HS = hardwareMap.get(CRServo.class, "HS");
        }
        public class ExtendSlide implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    HS.setPower(-0.3);
                    initialized = true;
                }

                double pos = AE.getVoltage();
                packet.put("HPos", pos);
                if (pos > 0.2) {
                    return true;
                } else {
                    HS.setPower(0);
                    return false;
                }
            }
        }
        public Action extendSlide() {
            return new ExtendSlide();
        }

        public class RetractSlide implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    HS.setPower(0.3);
                    initialized = true;
                }

                double pos = AE.getVoltage();
                packet.put("HPos", pos);
                if (pos < 2.32) {
                    return true;
                } else {
                    HS.setPower(0);
                    return false;
                }
            }
        }
        public Action retractSlide() {
            return new RetractSlide();
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

        Pose2d beginPose = new Pose2d(0, -61, Math.toRadians(270));



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Pose2d beginPose = new Pose2d(8, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        SlapGripper GR2 = new SlapGripper(hardwareMap);

        Horizontal HS = new Horizontal(hardwareMap);
        SlapWrist WR2 = new SlapWrist(hardwareMap);
        SlapArm ARM = new SlapArm(hardwareMap);



        Actions.runBlocking(GR2.closeGripper());

        Actions.runBlocking(WR2.chamberFlip());

        Actions.runBlocking(new SleepAction(1));

        Actions.runBlocking(ARM.initArm());
        //Actions.runBlocking(HS.retractSlide());




        Action chamber1 = drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(0, -35.5), Math.toRadians(90))
                .build();
        Action back1 = drive.actionBuilder(new Pose2d(0, -35.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(90))
                .build();


        Action zone1 = drive.actionBuilder(new Pose2d(0, -30, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(48, -48), Math.toRadians(180))
                .build();

        Action grab1 = drive.actionBuilder(new Pose2d(48, -48, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(48, -58), Math.toRadians(90))
                .build();

        Action chamber2 = drive.actionBuilder(new Pose2d(48, -58, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-5, -35.5), Math.toRadians(90))
                .build();

        Action back2 = drive.actionBuilder(new Pose2d(-5, -35.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-5, -40), Math.toRadians(90))
                .build();

        Action block1 = drive.actionBuilder(new Pose2d(-5, -36, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(0, -48), Math.toRadians(90))
                .setTangent(Math.toRadians(22.5))
                .splineToLinearHeading(new Pose2d(36, -36, Math.toRadians(-90)), Math.toRadians(22.5))
                .setTangent(Math.toRadians(90-12.5))
                .splineToConstantHeading(new Vector2d(39, -12), Math.toRadians(270+45))
                .setTangent(270-45)
                .splineToConstantHeading(new Vector2d(52, -50), Math.toRadians(90+45))

                .setTangent(Math.toRadians(90+45))
                .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(270+45))

                .splineToConstantHeading(new Vector2d(58, -24), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(64, -60), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(48, -45), Math.toRadians(270))
                .build();
        Action grab2 = drive.actionBuilder(new Pose2d(48, -45, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(48, -58), Math.toRadians(90))
                .build();
        Action chamber3 = drive.actionBuilder(new Pose2d(48, -58, Math.toRadians(270)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5, -35.5), Math.toRadians(90))
                .build();


        waitForStart();
        runtime.reset();



        Actions.runBlocking(
                new SequentialAction (
                        ARM.wallArm1(),
                        chamber1,
                        new SleepAction(0.3),
                        WR2.chamberFlip(),
                        new SleepAction(0.4),
                        ARM.chamberArm(),
                        new SleepAction(0.5),
                        GR2.openGripper(),
                        back1,
                        new SleepAction(0.5),
                        ARM.wallArm1(),
                        zone1,
                        ARM.wallArm(),
                        WR2.wallFlip(),
                        new SleepAction(1),
                        grab1,
                        new SleepAction(1),
                        GR2.closeGripper(),
                        new SleepAction(1),
                        ARM.wallArm1(),
                        new SleepAction(1),
                        chamber2,
                        new SleepAction(0.5),
                        WR2.chamberFlip(),
                        new SleepAction(0.2),
                        ARM.chamberArm(),
                        new SleepAction(0.5),
                        GR2.openGripper(),
                        back2,
                        new SleepAction(0.5),
                        ARM.wallArm1(),
                        new SleepAction(0.5),
                        block1,

                        ARM.wallArm(),
                        WR2.wallFlip(),
                        new SleepAction(1),
                        grab2,
                        new SleepAction(1),
                        GR2.closeGripper(),
                        new SleepAction(1),
                        ARM.wallArm1(),
                        chamber3,
                        new SleepAction(1),
                        WR2.chamberFlip(),
                        new SleepAction(0.2),
                        ARM.chamberArm()
                )
        );


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

}
