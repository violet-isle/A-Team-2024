

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
@Autonomous(name="HangAllAuto24")
public class HangAllAuto24 extends LinearOpMode {

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


    public class SlapGripper {
        private Servo GR2;

        public SlapGripper(HardwareMap hardwareMap) {
            GR2 = hardwareMap.get(Servo.class, "GR2");
        }
        public class CloseGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR2.setPosition(0);
                return false;
            }
        }
        public Action closeGripper() {
            return new CloseGripper();
        }

        public class OpenGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                GR2.setPosition(0.27);
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
                ARM.setPosition(0);
                return false;
            }
        }
        public Action chamberArm() {
            return new ChamberArm();
        }

        public class WallArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ARM.setPosition(0.27);
                return false;
            }
        }
        public Action wallArm() {
            return new WallArm();
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
                if (pos < 3.1) {
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

        Pose2d beginPose = new Pose2d(25, -66, Math.toRadians(90));



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Pose2d beginPose = new Pose2d(8, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Gripper GR = new Gripper(hardwareMap);

        Horizontal HS = new Horizontal(hardwareMap);

        Lift lift = new Lift(hardwareMap);

        Actions.runBlocking(GR.closeGripper());
        Actions.runBlocking(HS.retractSlide());


        TrajectoryActionBuilder chamber= drive.actionBuilder(beginPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, -38, Math.toRadians(90)), Math.toRadians(90));

        Action block1 = chamber.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(0, -48), Math.toRadians(90))
                .setTangent(Math.toRadians(22.5))
                .splineToLinearHeading(new Pose2d(40, -36, Math.toRadians(-90)), Math.toRadians(22.5))
                .setTangent(Math.toRadians(90-12.5))
                .splineToConstantHeading(new Vector2d(45, -12), Math.toRadians(270+45),
                    null,
                    new ProfileAccelConstraint(-5,10))
                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(90+45),
                    null,
                    new ProfileAccelConstraint(-5,10))
                .setTangent(Math.toRadians(90+45))
                .splineToConstantHeading(new Vector2d(58, 0), Math.toRadians(270+45),
                        null,
                        new ProfileAccelConstraint(-5,10))

                .splineToConstantHeading(new Vector2d(64, -24), Math.toRadians(270),
                        null,
                        new ProfileAccelConstraint(-5,10))

                .splineToConstantHeading(new Vector2d(64, -60), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(64, -50), Math.toRadians(270))
                .build();

        Action grab1 = drive.actionBuilder(new Pose2d(64, -50, Math.toRadians(270)))

                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(66, -64), Math.toRadians(0))
                .build();
        Action chamber2 = drive.actionBuilder(new Pose2d(66, -64, Math.toRadians(0)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(9, -34.5, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action back = drive.actionBuilder(new Pose2d(9, -34.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90))
                .build();

        Action grab2 = drive.actionBuilder(new Pose2d(9, -48, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(0, -48), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(0)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(64, -64), Math.toRadians(0))
                .build();
        Action chamber3 = drive.actionBuilder(new Pose2d(64, -64, Math.toRadians(0)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-9, -34.5, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action back2 = drive.actionBuilder(new Pose2d(-9, -34.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(9, -48), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(65, -65), Math.toRadians(0))
                .build();


        waitForStart();
        runtime.reset();

        Action chamberAction;
        chamberAction = chamber.build();



        Actions.runBlocking(
                new SequentialAction(
                        lift.liftUp(),
                        chamberAction,
                        lift.liftDown(),
                        GR.openGripper(),
                        block1,
                        lift.liftMoreDown(),
                        grab1,
                        GR.closeGripper(),
                        new SleepAction(1),
                        lift.liftUp(),
                        chamber2,
                        lift.liftDown(),
                        GR.openGripper(),
                        back,
                        lift.liftMoreDown(),
                        grab2,
                        GR.closeGripper(),
                        lift.liftUp(),
                        chamber3,
                        lift.liftDown(),
                        GR.closeGripper(),
                        back2
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
