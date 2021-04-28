package org.firstinspires.ftc.teamcode.vision;
import android.annotation.SuppressLint;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.math.BigDecimal;


@Autonomous
public class auto_framework extends LinearOpMode implements SensorEventListener
{
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    CRServo servo;
    static final double COUNTS_PER_MOTOR_REV = 96 * 60 / 3.5;    // eg: TETRIX Motor Encoder. 96 @ 1:1 * 60:1 / 3 measured adjustment value
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED    = 0.5;
    static final double TURN_90_DEGREE = 20; // calibrated value for the robot to go 90 degrees
    static final double DRIFT_VARIABLE = 0.76;
    private ElapsedTime runtime = new ElapsedTime();
    private int tick = 21; // calibrated value for the robot to go through one block

    // System time
    long lastSensorUpdateSysTime = System.currentTimeMillis();
    long lastGyroChangeTime = System.currentTimeMillis();

    // Constants / Params
    final float LPF_ALPHA = 0.25f; // LPF filter Alpha
    final boolean USE_LPF_FILTER = true;

    // Raw sensor data record
    float[] calMagDataRaw = {0, 0, 0};
    float[] gyroDataRaw = {0, 0, 0};
    double[] gyroAng = {0, 0, 0};
    double[] magAng = {0, 0, 0};

    // Sensor definitions / variables
    SensorManager sensorManager;

    @Override
    public void runOpMode()
    {
        initHardware();
        initPhoneSensors();
        initVision();

        waitForStart();

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        sleep(2000);

        // Don't burn CPU cycles busy-looping in this sample
        if(pipeline.position.equals(SkystoneDeterminationPipeline.RingPosition.NONE)){
            // Trajectory A
            trajA();
        }else if(pipeline.position.equals(SkystoneDeterminationPipeline.RingPosition.ONE)){
            // Trajectory B
            trajB();
        }else if(pipeline.position.equals(SkystoneDeterminationPipeline.RingPosition.FOUR)) {
            // Trajectory C
            trajC();
        }
        telemetry.addLine("Finish wobble goal");
        telemetry.update();

    }

    // Trajectory A
    public void trajA() {
        //drive to the line
        encoderDrive(FORWARD_SPEED,4*tick,4*tick,10);
        //drop the wobble
        servo.setPower(-1);
        while(runtime.time()<1.0){
            telemetry.addLine("Servo is turning");
            telemetry.addData("time", runtime.seconds());
            telemetry.update();
        }
        servo.setPower(0);
    }

    // Trajectory B
    public void trajB() {
        //go to the line and turn
//        encoderDrive(FORWARD_SPEED,4*tick, 4*tick, 10); // This sends it to A
        encoderDrive(FORWARD_SPEED,5*tick, 5*tick, 10); // This sends to B
        encoderTurn(TURN_SPEED, TURN_90_DEGREE,-1,10); // assume this translates
        /* The car theoritically turns in a way that would allow it to drop the wobble goal
         * into the B Block while staying on the line so that it can park right there and not
         * have to move afterwards. */
        /* The car now should move up to Block B's row and then translate over to block B
         * after dropping the wobble goal it should then move backwards onto the midline */
        /*This is all assuming that the car still works the same or in a "similar" way as before
        Steven redoes everything. */
        servo.setPower(-1); // drops wobble goal
        while(runtime.time()<1.0){ // one second run time total
            telemetry.addLine("Servo is turning");
            telemetry.addData("time", runtime.seconds());
            telemetry.update();
        }
        servo.setPower(0); // closes the wobble hand
        encoderDrive(FORWARD_SPEED,-1*tick, -1*tick, 10); // moves back to midline
    }

    // Trajectory C
    public void trajC() {
        //drive to the box
        encoderDrive(FORWARD_SPEED,6*tick, 6*tick, 10);
        //drop the wobble
        servo.setPower(-1);
        while(runtime.time()<1.0){
            telemetry.addLine("Servo is turning");
            telemetry.addData("time", runtime.seconds());
            telemetry.update();
        }
        servo.setPower(0);
        //go back to the midline
        encoderDrive(FORWARD_SPEED,-2*tick, -2*tick, 10);
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
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

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

    public void encoderTurn(double speed, double inches, int direction, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = leftFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(direction*newTarget);
            rightFront.setTargetPosition(-direction*newTarget);
            leftBack.setTargetPosition(-direction*newTarget);
            rightBack.setTargetPosition(direction*newTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion. *Auto2Jerjer.DRIFT_VARIABLE
            runtime.reset();
            leftFront.setPower(direction*Math.abs(speed));
            rightFront.setPower(direction*Math.abs(speed));
            leftBack.setPower(direction*Math.abs(speed));
            rightBack.setPower(direction*Math.abs(speed));

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
                telemetry.addData("EncoderTurnDrive Running", "");
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

    @SuppressLint("SetTextI18n")
    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            long tDiff = System.currentTimeMillis() - lastGyroChangeTime;
            lastGyroChangeTime = System.currentTimeMillis();
            gyroDataRaw = lowPass(event.values.clone(), gyroDataRaw);
            gyroAng[0] += tDiff / 1000.0 * gyroDataRaw[0];
            gyroAng[1] += tDiff / 1000.0 * gyroDataRaw[1];
            gyroAng[2] += tDiff / 1000.0 * gyroDataRaw[2];
            gyroDataRaw = event.values;
        }
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            if (USE_LPF_FILTER)
                calMagDataRaw = lowPass(event.values.clone(), calMagDataRaw);
            else
                calMagDataRaw = event.values.clone();
            magAng[0] = (float) Math.toDegrees(Math.atan2(calMagDataRaw[0], calMagDataRaw[1]));
            magAng[1] = (float) Math.toDegrees(Math.atan2(calMagDataRaw[1], calMagDataRaw[2]));
            magAng[2] = (float) Math.toDegrees(Math.atan2(calMagDataRaw[2], calMagDataRaw[0]));
        }
        // Update time
        lastSensorUpdateSysTime = System.currentTimeMillis();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Unused
    }

    public float[] lowPass(float[] input, float[] output) {
        if (output == null) return input;
        for (int i = 0; i < input.length; i++) {
            output[i] = output[i] + LPF_ALPHA * (input[i] - output[i]);
        }
        return output;
    }

    public BigDecimal[] round(float[] d, int decimalPlace) {
        BigDecimal[] bdarr = new BigDecimal[d.length];
        for (int i = 0; i < d.length; i++) {
            BigDecimal bd = new BigDecimal(d[i] + "");
            bdarr[i] = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
        }
        return bdarr;
    }

    public BigDecimal round(float d, int decimalPlace) {
        BigDecimal bd = new BigDecimal(d + "");
        bd = bd.setScale(decimalPlace, BigDecimal.ROUND_HALF_UP);
        return bd;
    }

    public void resetGyroAng() {
        gyroAng = new double[]{0, 0, 0};
    }

    public void initVision () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    public void initPhoneSensors () {
        sensorManager.registerListener((SensorEventListener) this,
                sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener((SensorEventListener) this,
                sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);
    }

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
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.get(CRServo.class, "servo");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(120,140);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1 = -1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}