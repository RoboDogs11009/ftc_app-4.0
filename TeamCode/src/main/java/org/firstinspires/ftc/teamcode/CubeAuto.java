package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

//john paul is not fun
//josh is fun
@Autonomous(name = "GoldAuto", group = "Sensor")
                    // Comment this out to add to the opmode list
public class CubeAuto extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor Lf;
    private DcMotor Lb;
    private DcMotor Rf;
    private DcMotor Rb;
    private DcMotor Li;



    GoldAlignDetector detector = new GoldAlignDetector();
    Hardware robot = new Hardware();



    //VU mark
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
//end vu mark

    double strifePower = 0.4;

   // double distance;
   // double colorRed;
    //double colorBlue;
    double drivePower = .7;
    double mineralPosition = 2;



    //Encoder Variables
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.75 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


   // DistanceSensor sensorDistance;



   // DistanceSensor rightDistance;
   // DistanceSensor leftDistance;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

   // ModernRoboticsI2cRangeSensor rangeSensor;



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void lift (double drive, double timer) {
        robot.Li.setPower(drive);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timer)) {
            telemetry.addData("Lowering", 0);

            telemetry.addData("RunTime", runtime.seconds());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.Li.setPower(0);
    }

    public void turnLeft(double drive, double timer) {

        robot.Lf.setPower(-drive);
        robot.Lb.setPower(-drive);
        robot.Rf.setPower(drive);
        robot.Rb.setPower(drive);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timer)) {

            telemetry.addData("Turning Left", 0);
            // send the info back to driver station using telemetry function.

            telemetry.addData("RunTime", runtime.seconds());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();


        }
        drive(0,.01);



    } // End Turn Left

    public void turnRight(double drive, double timer) {

        robot.Lf.setPower(drive);
        robot.Lb.setPower(drive);
        robot.Rf.setPower(-drive);
        robot.Rb.setPower(-drive);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timer)) {

            telemetry.addData("Turning Right", 0);
            // send the info back to driver station using telemetry function.

            telemetry.addData("RunTime", runtime.seconds());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();


        }

        drive(0,.01);

    }

    public void drive(double drive, double timer) {

        robot.Lf.setPower(drive);
        robot.Lb.setPower(drive);
        robot.Rf.setPower(drive);
        robot.Rb.setPower(drive);


        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timer)) {

            telemetry.addData("Driving Forward", 0);
            // send the info back to driver station using telemetry function.

            telemetry.addData("RunTime", runtime.seconds());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();


        }
        robot.Lf.setPower(0);
        robot.Lb.setPower(0);
        robot.Rf.setPower(0);
        robot.Rb.setPower(0);
    } //End Drive

    public void waiting(double timer) {




        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timer)) {

            telemetry.addData("Waiting", 0);
            // send the info back to driver station using telemetry function.

            telemetry.addData("RunTime", runtime.seconds());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();


        }
    } //End Wait

    public void gyro(double degree, double timer)
    {


        //----------------------------------------------------------------------------------------------
        // Main logic
        //----------------------------------------------------------------------------------------------
        double leftFront = 0;
        double leftBack = 0;
        double rightFront = 0;
        double rightBack = 0;
        double difference = 0;



        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timer) ) {

            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            AngleUnit angleUnit = angles.angleUnit;
            double angle = angles.firstAngle;


            double controlAngle = degree;

            double gyroAngle = AngleUnit.DEGREES.fromUnit(angleUnit, angle);

            if (gyroAngle < controlAngle)
            {
                difference = controlAngle - gyroAngle;

                leftFront = -difference * .01;
                leftBack = -difference * .01;
                rightFront = difference * .01;
                rightBack = difference * .01;
                telemetry.addData("Turning right", 0);
                telemetry.addData("Difference", difference);

            }
            if ((controlAngle > 90) && (gyroAngle < -90))
            {
                difference = controlAngle - gyroAngle;
                leftFront = difference * .001;
                leftBack = difference * .001;
                rightFront = -difference * .001;
                rightBack = -difference * .001;
                telemetry.addData("Turning right", 0);
                telemetry.addData("Difference", difference);
            }

            if (controlAngle < gyroAngle)
            {
                difference = controlAngle - gyroAngle;

                leftFront = -difference * .01;
                leftBack = -difference * .01;
                rightFront = difference * .01;
                rightBack = difference * .01;
                telemetry.addData("Turning left", 0);
                telemetry.addData("Difference", difference);

            }

            if ((controlAngle < -90) && (gyroAngle > 90))
            {
                difference = controlAngle - gyroAngle;
                leftFront = difference * .01;
                leftBack = difference * .01;
                rightFront = -difference * .01;
                rightBack = -difference * .01;
                telemetry.addData("Turning right", 0);
                telemetry.addData("Difference", difference);
            }

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)




            robot.Lf.setPower(Range.clip(leftFront,-.3,.3));
            robot.Lb.setPower(Range.clip(leftBack,-.3,.3));
            robot.Rf.setPower(Range.clip(rightFront,-.3,.3));
            robot.Rb.setPower(Range.clip(rightBack,-.3,.3));



            // Send telemetry message to signify robot running;

            telemetry.addData("left",  "%.2f", leftFront);
            telemetry.addData("left",  "%.2f", leftBack);
            telemetry.addData("right", "%.2f", rightFront);
            telemetry.addData("right", "%.2f", rightBack);

            telemetry.addData("Control Angle", controlAngle);
            telemetry.addData("Robot Angle", gyroAngle);
            telemetry.addData("Difference", difference);



        }

        robot.Lf.setPower(0);
        robot.Lb.setPower(0);
        robot.Rf.setPower(0);
        robot.Rb.setPower(0);

    }  //END GYRO

    public void gyroInit(){
        //GYRO
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void encoderDrive (double leftDistance, double rightDistance, double MAXSpeed, double timeOut){

//80.4937 counts per inch
        //RESET ENCODER

        // Send telemetry message to signify robot waiting;
        telemetry.addData("ENCODERING!!!!!!!!!", 0);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.Lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.Lf.getCurrentPosition(),
                robot.Lb.getCurrentPosition(),
                robot.Rf.getCurrentPosition(),
                robot.Rb.getCurrentPosition());
        telemetry.update();

        encoder(leftDistance,  rightDistance, MAXSpeed,timeOut);  // S1: Power, Left, Right, Time
    }

    public void encoder (
            double leftInches, double rightInches, double maxSpeed,
            double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        int currPosLeftFront;
        int currPosLeftBack;
        int currPosRightFront;
        int currPosRightBack;

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double leftFrontIntregal = 0;
        double leftBackIntregal = 0;
        double rightFrontIntregal = 0;
        double rightBackIntregal = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.Lf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.Lb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.Rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.Rb.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.Lf.setTargetPosition(newLeftFrontTarget);
            robot.Lb.setTargetPosition(newLeftBackTarget);
            robot.Rf.setTargetPosition(newRightFrontTarget);
            robot.Rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.Lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS && robot.Lf.isBusy() && robot.Lb.isBusy() && robot.Rf.isBusy() && robot.Rb.isBusy()) )
            {
                currPosLeftFront = robot.Lf.getCurrentPosition();
                currPosLeftBack = robot.Lb.getCurrentPosition();
                currPosRightFront = robot.Rf.getCurrentPosition();
                currPosRightBack = robot.Rb.getCurrentPosition();


                leftFrontPower = (newLeftFrontTarget - currPosLeftFront)/200;
                leftBackPower = (newLeftBackTarget - currPosLeftBack)/200;
                rightFrontPower = (newRightFrontTarget - currPosRightFront)/200;
                rightBackPower = (newRightBackTarget - currPosRightBack)/200;

                //Left Front Intregal
                if (robot.Lf.isBusy() == true)
                {
                    if ((newLeftFrontTarget - currPosLeftFront) > 0) {
                        leftFrontIntregal = leftFrontIntregal + .001;
                    } else {
                        leftFrontIntregal = 0;

                    }
                }
                //Left Back Intregal
                if (robot.Lb.isBusy() == true)
                {
                    if ((newLeftBackTarget - currPosLeftBack) > 0) {
                        leftBackIntregal = leftBackIntregal + .001;
                    } else {
                        leftBackIntregal = 0;
                    }
                }

                //Right Front Intregal
                if (robot.Rf.isBusy() == true)
                {
                    if ((newRightFrontTarget - currPosRightFront) > 0) {
                        rightFrontIntregal = rightFrontIntregal + .001;
                    } else {
                        rightFrontIntregal = 0;
                    }
                }

                //Right Back Intregal
                if (robot.Rb.isBusy() == true)
                {
                    if ((newLeftBackTarget - currPosRightBack) > 0) {
                        rightBackIntregal = rightBackIntregal + .001;
                    } else {
                        rightBackIntregal = 0;
                    }
                }

                robot.Lf.setPower(Math.abs(Range.clip(leftFrontPower, -maxSpeed, maxSpeed) + Range.clip(leftFrontIntregal, 00, .1)));
                robot.Lb.setPower(Math.abs(Range.clip(leftBackPower, -maxSpeed, maxSpeed) + Range.clip(leftBackIntregal, 00, .1)));
                robot.Rf.setPower(Math.abs(Range.clip(rightFrontPower, -maxSpeed, maxSpeed) + Range.clip(rightFrontIntregal, 0, .1)));
                robot.Rb.setPower(Math.abs(Range.clip(rightBackPower, -maxSpeed, maxSpeed) + Range.clip(rightBackIntregal, 0, .1)));

                // Display it for the driver.
                telemetry.addData("ENCODERING!!!!!!!!!", 0);
                telemetry.addData("LF Left", newLeftFrontTarget-currPosLeftFront);
                telemetry.addData("LB Left", newLeftBackTarget-currPosLeftBack);
                telemetry.addData("RF Left", newRightFrontTarget-currPosRightFront);
                telemetry.addData("RB Left", newRightBackTarget-currPosRightBack);
                telemetry.addData("LF Power", leftFrontPower);
                telemetry.addData("LB Power",leftBackPower );
                telemetry.addData("RF Power", rightFrontPower);
                telemetry.addData("RB Power", rightBackPower);

                telemetry.update();
                sleep(20);
            }

            drive(0, .1);

            // Turn off RUN_TO_POSITION
            robot.Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot. Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Li.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    public void strafeLeft (double power, double timer){
        robot.Rb.setPower(-power);
        robot.Rf.setPower(power);
        robot.Lb.setPower(power);
        robot.Lf.setPower(-power);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timer)) {
            telemetry.addData("Path", "Strafing Left: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

   public void liftEncoder (int counts, double timer){
        robot.Li.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Ra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.La.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        robot.Li.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.La.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Ra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.Li.setTargetPosition(counts);
        robot.La.setTargetPosition(counts);
        robot.Ra.setTargetPosition(counts);
        robot.Li.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.La.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Ra.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.Li.setPower(1);
        robot.La.setPower(1);
        robot.Ra.setPower(1);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < timer && robot.Li.isBusy() )){
            telemetry.addData("Lift Encoder", robot.Li.getCurrentPosition());
            telemetry.update();
        }

        robot.Li.setPower(0);
        robot.La.setPower(0);
        robot.Ra.setPower(0);

       robot.Li.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       robot.La.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       robot.Ra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


   }

    // ++ means right
    public void strafeEncoder (
            double inches,
            double timeoutS) {

        double leftInches = inches;
        double rightInches = inches;

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        int currPosLeftFront;
        int currPosLeftBack;
        int currPosRightFront;
        int currPosRightBack;

        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;

        double leftFrontIntregal = 0;
        double leftBackIntregal = 0;
        double rightFrontIntregal = 0;
        double rightBackIntregal = 0;

        robot.Lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = Lf.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = Lb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = Rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = Rb.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            robot.Lf.setTargetPosition(newLeftFrontTarget);
            robot.Lb.setTargetPosition(newLeftBackTarget);
            robot.Rf.setTargetPosition(newRightFrontTarget);
            robot.Rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.Lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) )
            {
                currPosLeftFront = robot.Lf.getCurrentPosition();
                currPosLeftBack = robot.Lb.getCurrentPosition();
                currPosRightFront = robot.Rf.getCurrentPosition();
                currPosRightBack = robot.Rb.getCurrentPosition();


                leftFrontPower = (newLeftFrontTarget - currPosLeftFront)/250;
                leftBackPower = (newLeftBackTarget - currPosLeftBack)/250;
                rightFrontPower = (newRightFrontTarget - currPosRightFront)/250;
                rightBackPower = (newRightBackTarget - currPosRightBack)/250;

                //Left Front Intregal
                if (robot.Lf.isBusy() == true)
                {
                    if ((newLeftFrontTarget - currPosLeftFront) > 0) {
                        leftFrontIntregal = leftFrontIntregal + .01;
                    } else {
                        leftFrontIntregal = 0;

                    }
                }
                //Left Back Intregal
                if (robot.Lb.isBusy() == true)
                {
                    if ((newLeftBackTarget - currPosLeftBack) > 0) {
                        leftBackIntregal = leftBackIntregal + .01;
                    } else {
                        leftBackIntregal = 0;
                    }
                }

                //Right Front Intregal
                if (robot.Rf.isBusy() == true)
                {
                    if ((newRightFrontTarget - currPosRightFront) > 0) {
                        rightFrontIntregal = rightFrontIntregal + .01;
                    } else {
                        rightFrontIntregal = 0;
                    }
                }

                //Right Back Intregal
                if (robot.Rb.isBusy() == true)
                {
                    if ((newLeftBackTarget - currPosRightBack) > 0) {
                        rightBackIntregal = rightBackIntregal + .01;
                    } else {
                        rightBackIntregal = 0;
                    }
                }


                robot.Lf.setPower(Math.abs( Range.clip(leftFrontPower, -.15, .15)));
                robot.Lb.setPower(Math.abs( Range.clip(leftBackPower, -.15, .15)));
                robot.Rf.setPower(Math.abs( Range.clip(rightFrontPower, -.15, .15)));
                robot.Rb.setPower(Math.abs( Range.clip(rightBackPower, -.15, .15)));

                // Display it for the driver.
                telemetry.addData("ENCODERING Strafe Style!!!!!!!!!", 0);
                telemetry.addData("LF Left", newLeftFrontTarget-currPosLeftFront);
                telemetry.addData("LB Left", newLeftBackTarget-currPosLeftBack);
                telemetry.addData("RF Left", newRightFrontTarget-currPosRightFront);
                telemetry.addData("RB Left", newRightBackTarget-currPosRightBack);
                telemetry.addData("LF Power", leftFrontPower);
                telemetry.addData("LB Power",leftBackPower );
                telemetry.addData("RF Power", rightFrontPower);
                telemetry.addData("RB Power", rightBackPower);

                telemetry.update();
                sleep(20);
            }

            drive(0, .1);

            // Turn off RUN_TO_POSITION
            robot.Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Li.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }
    }

    public void pause ()
    {

        runtime.reset();
        while (opModeIsActive() && (!gamepad1.a)) {

            telemetry.addData("Pause", 0);
            // send the info back to driver station using telemetry function.

            telemetry.addData("RunTime", runtime.seconds());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void servo (double position1, double position2, double timer)
    {

         robot.S.setPosition(position1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timer)) {

            telemetry.addData("Servo", position1);
            // send the info back to driver station using telemetry function.

            telemetry.addData("RunTime", runtime.seconds());
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.S.setPosition(position2);
    }

    public void detectMineral(double timer, double gyroLimit, double turnPower){

        double xPos=0;

     detector.enable();
     sleep(2000);

     runtime.reset();
     while (opModeIsActive() && runtime.seconds() < timer ) {

         xPos = detector.getXPosition();

         telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
         telemetry.addData("IsAligned", detector.getAligned());
         telemetry.addData("X Pos", detector.getXPosition());
         telemetry.addData("Power", turnPower);
         telemetry.update();
     }
     detector.disable();


        if (xPos <= 340) {
            mineralPosition = 2;
        }
        if (xPos > 340){
            mineralPosition = 1;
        }
        if (xPos == 0){
            mineralPosition = 0;
        }
    }

    public void detectorInit() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment




    }

    //End Drive
    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);






        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

        // get a reference to the color sensor.


        // get a reference to the distance sensor that shares the same name.


        waitForStart();

        //Timer Reset
        runtime.reset();
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            liftEncoder(12000, 5);

            drive(-.2,1.5);
            gyroInit();
             gyro(20,2);
             liftEncoder(-2000,1);


            detectorInit();
            detectMineral(1, 50, 0.15);

            if (mineralPosition == 0){
                gyro(-30, 1);
                encoderDrive(40, 40, .5, 3 );
                gyro(-90, 2);
                encoderDrive(-22,-22, .4, 4);
                servo(0,1,2);
                encoderDrive(-18,-18, .4, 4);
                gyro(-47,2);
                encoderDrive(-55,-55, .5, 4);


            }
            if (mineralPosition == 1){
                gyro(0, 1);
                encoderDrive(40, 40, .5, 3 ); //CHANGE
                gyro(-90, 2);
               //encoderDrive(-22,-22, .4, 4); //CHANGE??????????
                servo(0,1,2);
                encoderDrive(-18,-18, .4, 4);//CHANGE
                gyro(-47,2);
                encoderDrive(-55,-55, .5, 4);

            }
            if (mineralPosition == 2){
               gyro(30, 1);
                encoderDrive(40, 40, .5, 3 );
                gyro(-90, 2);
                encoderDrive(16,16, .4, 4);
                servo(0,1,1.5);
                encoderDrive(-18,-18, .4, 4);
                gyro(-47,2);
                encoderDrive(-55,-55, .5, 4);

            }




            waiting(30);
       }

    } // END RUN OP MODE
} // END CLASS