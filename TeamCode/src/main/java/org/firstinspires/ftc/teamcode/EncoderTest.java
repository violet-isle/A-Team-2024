

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Encoder Test")
public class EncoderTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;


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

        VS1 = hardwareMap.get(DcMotorEx.class, "VS1");
        VS2 = hardwareMap.get(DcMotorEx.class, "VS2");

        leftDeadWheel  = hardwareMap.get(DcMotor.class, "FL");
        rightDeadWheel  = hardwareMap.get(DcMotor.class, "BR");
        backDeadWheel  = hardwareMap.get(DcMotor.class, "FR");

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // Send calculated power to wheels
            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);

            if(gamepad1.dpad_up){
                VSTargetPos++;
            }

            if(gamepad1.dpad_down){
                VSTargetPos--;
            }



            if(gamepad1.triangle){
                VS1.setTargetPosition(VSTargetPos);
                VS2.setTargetPosition(VSTargetPos);
                VS1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                VS2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) VS1).setVelocity(100);
                ((DcMotorEx) VS2).setVelocity(100);
            }
            VSPos = VS1.getCurrentPosition();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.addData("Back Deadwheel", backDeadWheel.getCurrentPosition());
            telemetry.addData("Right Deadwheel", rightDeadWheel.getCurrentPosition());
            telemetry.addData("left Deadwheel", leftDeadWheel.getCurrentPosition());


            telemetry.addData("VS1", VS1.getCurrentPosition());
            telemetry.addData("VS2", VS2.getCurrentPosition());

            telemetry.addData("VS1T", VS1.getTargetPosition());
            telemetry.addData("VS2T", VS2.getTargetPosition());
            telemetry.addData("VSPos", VSPos);

            telemetry.addData("VSTargetPos", VSTargetPos);
            telemetry.update();
        }
    }}
