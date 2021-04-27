package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;



@TeleOp(name="Demo Drive Controls", group="Iterative Opmode")
//@Disabled
public class Drive_Control extends LinearOpMode {

    // Declare vars (components)
    //motors are defined if taken from a top down view
    DcMotor intakeGreen;
    DcMotor intakeBlack;
    DcMotor ShooterRight;
    DcMotor ShooterLeft;

    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    CRServo servo;
    int dpishift = 1;
    double contPower;
    static final double COUNTS_PER_MOTOR_REV = 180;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 3.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED    = 0.5;
    static final double TURN_90_DEGREE = 20; // calibrated value for the robot to go 90 degrees
    static final double DRIFT_VARIABLE = 0.76;
    private ElapsedTime runtime = new ElapsedTime();
    private int tick = 21; // calibrated value for the robot to go through one block


    ColorSensor colorSensor;


    @Override
    public void runOpMode(){
        initHardware();

        servo = hardwareMap.get(CRServo.class, "servo");
        colorSensor = hardwareMap.colorSensor.get("sensor");

        waitForStart();

        while(opModeIsActive()) {

            //start reading color sensor
            colorSensor.enableLed(true);



            // intake
            if (gamepad2.right_bumper) {
                intakeGreen.setPower(1);
                intakeBlack.setPower(1);
            } else {
                intakeGreen.setPower(0);
                intakeBlack.setPower(0);
            }


            // shooter
            if (gamepad2.left_bumper) {
                ShooterLeft.setPower(1);
                ShooterRight.setPower(1);
            } else {
                ShooterRight.setPower(0);
                ShooterLeft.setPower(0);
            }

            // speed shift
            if (gamepad1.right_stick_x != 0) {

                if (gamepad1.left_bumper) {

                    dpishift = 2;
                }
                else {
                    dpishift = 1;
                }
            }

            int blueLineTimeoutTime = 5; //seconds
            if(gamepad2.b) {
                double startTimeSeconds = getRuntime();
                leftFront.setPower(0.3);
                rightFront.setPower(0.3);
                leftBack.setPower(0.3);
                rightBack.setPower(0.3);
                ShooterLeft.setPower(1);
                ShooterRight.setPower(1);
                while(colorSensor.blue() < 5 && getRuntime() - startTimeSeconds < blueLineTimeoutTime) {
                }
                if (colorSensor.blue() > 5) {
                    encoderDrive(0.1, -10, -10, 10);
                } else {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                }

                //align compass
            }


            // dpi shift hold
            if (gamepad1.left_bumper) {

                dpishift = 2;
            } else {

                dpishift = 1;
            }

            // servo
            contPower = gamepad1.left_trigger;
            servo.setPower(contPower);
            //telemetry.addData("servoPower", contPower);

            //color sensor reading
            telemetry.addData("Color Sensor reading: ", colorSensor.blue());
            telemetry.update();


            // Mecanum wheels
//            leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);
//            rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);
//            leftFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x)) / dpishift);
//            rightFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x)) / dpishift);

            // Mecanum wheel movememnts
            double r = Math.hypot(gamepad1.left_stick_x,gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x) - Math.PI / 4;
            double v1 = r * Math.cos(robotAngle) + gamepad1.right_stick_x;
            double v2 = r * Math.sin(robotAngle) - gamepad1.right_stick_x;
            double v3 = r * Math.sin(robotAngle) + gamepad1.right_stick_x;
            double v4 = r * Math.cos(robotAngle) - gamepad1.right_stick_x;
            double maxV = Math.max(Math.max(Math.abs(v1),Math.abs(v2)),Math.max(Math.abs(v3),Math.abs(v4)));
            if (maxV > 1) {
                v1 /= maxV;
                v2 /= maxV;
                v3 /= maxV;
                v4 /= maxV;
            }
            // dpi shift calculation
            v1 /= dpishift;
            v2 /= dpishift;
            v3 /= dpishift;
            v4 /= dpishift;
            // set motor power
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftBack.setPower(v3);
            rightBack.setPower(v4);

        }

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            leftBack.setTargetPosition(newLeftTarget);
            rightBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion. *Auto2Jerjer.DRIFT_VARIABLE
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed*DRIFT_VARIABLE));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed*DRIFT_VARIABLE));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                telemetry.clearAll();
                telemetry.addData("EncoderDrive Running", "");
                telemetry.addData("Positions: LF=" + leftFront.getCurrentPosition() + "  RF=" + rightFront.getCurrentPosition(), "");
                // Display it for the driver.
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);


            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    // Initializes hardware
    public void initHardware () {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeGreen = hardwareMap.get(DcMotor.class, "intakeGreen");
        intakeGreen.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeGreen.setDirection(DcMotor.Direction.FORWARD);
        intakeGreen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeGreen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeBlack = hardwareMap.get(DcMotor.class, "intakeBlack");
        intakeBlack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBlack.setDirection(DcMotor.Direction.REVERSE);
        intakeBlack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeBlack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterRight = hardwareMap.get(DcMotor.class, "ShooterRight");
        ShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterRight.setDirection(DcMotor.Direction.REVERSE);
        ShooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ShooterLeft = hardwareMap.get(DcMotor.class, "ShooterLeft");
        ShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterLeft.setDirection(DcMotor.Direction.REVERSE);
        ShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
