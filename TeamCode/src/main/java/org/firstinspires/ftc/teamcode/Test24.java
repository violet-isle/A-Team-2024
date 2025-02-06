

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Test24`")
public class Test24 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private CRServo HS;
    private Servo WR;
    private Servo GF;
    private Servo GR;

    private AnalogInput AE = null;

    private RevTouchSensor TS;


    private int VSPos;

    private int VSTargetPos;


    private DcMotorEx VS1 = null;
    private DcMotorEx VS2 = null;

    private DcMotor leftDeadWheel = null;
    private DcMotor rightDeadWheel = null;
    private DcMotor backDeadWheel = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");


        AE = hardwareMap.get(AnalogInput.class, "AE");


        WR = hardwareMap.get(Servo.class, "WR");
        HS = hardwareMap.get(CRServo.class, "HS");
        GR = hardwareMap.get(Servo.class, "GR");


        GF = hardwareMap.get(Servo.class, "UW");

        VS1 = hardwareMap.get(DcMotorEx.class, "VS1");
        VS2 = hardwareMap.get(DcMotorEx.class, "VS2");

        leftDeadWheel  = hardwareMap.get(DcMotor.class, "FL");
        rightDeadWheel  = hardwareMap.get(DcMotor.class, "BR");
        backDeadWheel  = hardwareMap.get(DcMotor.class, "FR");


        TS = hardwareMap.get(RevTouchSensor.class, "TS");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        VS1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VS2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        VS2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double servoInc = 0;
        double servoPos = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            if(gamepad2.square){servoInc += 0.0001;}
            if(gamepad2.circle){servoInc -= 0.0001;}

            if(gamepad2.triangle){servoPos += servoInc;}
            if(gamepad2.cross){servoPos -= servoInc;}


            if(gamepad2.dpad_left){
                GR.setPosition(servoPos);}
            if(gamepad2.dpad_up){
                WR.setPosition(servoPos);}
            if(gamepad2.dpad_right){
                GF.setPosition(servoPos);
                }





            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("Back Deadwheel", backDeadWheel.getCurrentPosition());
            telemetry.addData("Right Deadwheel", rightDeadWheel.getCurrentPosition());
            telemetry.addData("left Deadwheel", leftDeadWheel.getCurrentPosition());

            telemetry.addData("servo increment", servoInc);
            telemetry.addData("servo position", servoPos);

            telemetry.addData("GR2", GR.getPosition());
            telemetry.addData("AE", AE.getVoltage());
            telemetry.addData("GF", GF.getPosition());
            telemetry.addData("WR", WR.getPosition());



            telemetry.addData("TS", TS.isPressed());

            telemetry.addData("VS1", VS1.getCurrentPosition());
            telemetry.addData("VS2", VS2.getCurrentPosition());

            telemetry.addData("VS1T", VS1.getTargetPosition());
            telemetry.addData("VS2T", VS2.getTargetPosition());
            telemetry.addData("VSPos", VSPos);

            telemetry.addData("VSTargetPos", VSTargetPos);
            telemetry.update();
        }
    }}
