package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="rpmIncrement", group="Iterative Opmode")
public class rpmInc extends LinearOpMode {


    static final double COUNTS_PER_MOTOR_REV = 180;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 3.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double TURN_90_DEGREE = 20; // calibrated value for the robot to go 90 degrees
    static final double DRIFT_VARIABLE = 0.76;
    private ElapsedTime runtime = new ElapsedTime();
    private int tick = 21; // calibrated value for the robot to go through one block

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
    ColorSensor colorSensor;


//
//    public int runShooterAtRPM(double rpm, int initialSpeed) {
//        int speed = initialSpeed;
//        int newLeftTarget;
//        int newRightTarget;
//        double pastRPM = 0;
//
//        runtime.reset();
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = ShooterLeft.getCurrentPosition() + 360;
//            //newRightTarget = ShooterRight.getCurrentPosition() + 360;
//
//            double startTime = runtime.milliseconds();
//            ShooterLeft.setTargetPosition(newLeftTarget);
//            // ShooterRight.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            ShooterLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            //ShooterRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//            //ShooterRight.setPower(Math.abs(speed));
//            ShooterLeft.setPower(Math.abs(speed));
//
//
////
////            leftFront.setPower(Math.abs(speed));
////            rightFront.setPower(Math.abs(speed*DRIFT_VARIABLE));
////            leftBack.setPower(Math.abs(speed));
////            rightBack.setPower(Math.abs(speed*DRIFT_VARIABLE));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test
//
//
//           // while (opModeIsActive() &
//                   // (ShooterLeft.isBusy() && ShooterRight.isBusy())) {
//                telemetry.clearAll();
//                telemetry.addData("Past Rotation RPM: " + pastRPM, "");
//                telemetry.addData("RPM: Left=" + ShooterLeft.getCurrentPosition() + "  RPM: Right=" + ShooterRight.getCurrentPosition(), "");
//                // Display it for the driver.
//                telemetry.update();
//         //   }
//
//            pastRPM = (1 * 60) / (runtime.milliseconds() - startTime);
//            speed = (int) (((speed * (rpm / pastRPM) + speed)) / 2);
//
//
//            // Stop all motion;
////            leftFront.setPower(0);
////            rightFront.setPower(0);
////            leftBack.setPower(0);
////            rightBack.setPower(0);
//
//
//            // Turn off RUN_TO_POSITION
////            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//        return speed;
//    }

    @Override
    public void runOpMode() throws InterruptedException {

        int speed = 100;
        initHardware();
        servo = hardwareMap.get(CRServo.class, "servo");
        colorSensor = hardwareMap.get(ColorSensor.class,"sensor");

        waitForStart();

        //get current encoder position
        float leftShooter = ShooterLeft.getCurrentPosition();
        float rightShooter = ShooterRight.getCurrentPosition();

        double starttime = getRuntime();
        while(opModeIsActive()) {




//            //runtime check
//            if(getRuntime() - starttime % 200 == 0) {
//
//                wait(1);
//            }


            // intake
            if (gamepad2.right_bumper) {
                intakeGreen.setPower(1.0);
                intakeBlack.setPower(1.0);
            } else {
                intakeGreen.setPower(0.0);
                intakeBlack.setPower(0.0);
            }


            // shooter
            if (gamepad2.left_bumper) {
                ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ShooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ShooterLeft.setPower(1);
                ShooterRight.setPower(1);
            } else {
                ShooterRight.setPower(0.0);
                ShooterLeft.setPower(0.0);
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

            // dpi shift hold
            if (gamepad1.left_bumper) {

                dpishift = 2;
            } else {

                dpishift = 1;
            }

            // servo
            contPower = gamepad1.left_trigger;
            servo.setPower(contPower);
            telemetry.addData("servoPower", contPower);

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


            //RPM measuring
            telemetry.addData("left RPM: ",  ShooterLeft.getCurrentPosition() /48 );
            telemetry.addData("right RPM: ",  ShooterRight.getCurrentPosition() /48 );
            telemetry.update();
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
        intakeBlack.setDirection(DcMotor.Direction.FORWARD);
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
