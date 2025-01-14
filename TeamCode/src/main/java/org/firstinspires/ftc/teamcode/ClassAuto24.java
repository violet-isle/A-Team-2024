

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name="ClassAuto24")
public class ClassAuto24 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

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
                if (pos < 5300.0) {
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
                if (pos > 600) {
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
                if (pos > 50) {
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
                GR.setPosition(1.0);
                return false;
            }
        }
        public Action openGripper() {
            return new OpenGripper();
        }
    }

    public class Horizontal {
        private Servo HS;

        public Horizontal(HardwareMap hardwareMap) {
            HS = hardwareMap.get(Servo.class, "HS");
        }
        public class ExtendSlide implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HS.setPosition(1);
                return false;
            }
        }
        public Action extendSlide() {
            return new ExtendSlide();
        }
        public class ExtendSlide0 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HS.setPosition(0.3);
                return false;
            }
        }
        public Action extendSlide0() {
            return new ExtendSlide0();
        }

        public class RetractSlide implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HS.setPosition(0);
                return false;
            }
        }
        public Action retractSlide() {
            return new RetractSlide();
        }
    }

    private Servo WR;



    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftBack = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack = hardwareMap.get(DcMotor.class, "BR");


        LDW = hardwareMap.get(DcMotor.class, "FL");
        RDW = hardwareMap.get(DcMotor.class, "BR");
        BDW = hardwareMap.get(DcMotor.class, "FR");

        WR = hardwareMap.get(Servo.class, "WR");

        Pose2d beginPose = new Pose2d(28, -65.5, Math.toRadians(90));



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


        TrajectoryActionBuilder basket = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-45, -60, Math.toRadians(180+45/4)), Math.toRadians(180 + 45/2));

        Action block1 = basket.endTrajectory().fresh()
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(50, -43, Math.toRadians(90)), Math.toRadians(90))
                .build();

        waitForStart();
        runtime.reset();

        Action basketAction;
        basketAction = basket.build();



        Actions.runBlocking(
                new SequentialAction(
                        basketAction,
                        lift.liftUp(),
                        new SleepAction(1),
                        HS.extendSlide(),
                        new SleepAction(0.5),
                        GR.openGripper(),
                        new SleepAction(0.3),
                        HS.retractSlide(),
                        new SleepAction(0.5),
                        lift.liftDown(),
                        block1,
                        HS.extendSlide0(),
                        new SleepAction(0.5),
                        lift.liftMoreDown(),
                        new SleepAction(0.5),
                        GR.closeGripper(),
                        new SleepAction(0.5),
                        lift.liftDown(),
                        basketAction
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
