package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.util.Range.scale;
import static java.lang.Math.abs;


@TeleOp(name="yesDrive", group="Linear Opmode")
//@Disabled
public class yesDrive extends OpMode {
 
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Lf = null;
    private DcMotor Lb = null;
    private DcMotor Rf = null;
    private DcMotor Rb = null;
    private DcMotor Li = null;
    private DcMotor Ra = null;
    private DcMotor La = null;
    private DcMotor El = null;

    private CRServo L  = null;
    private CRServo R  = null;

    private Servo Lw   = null;
    private Servo Rw   = null;

    private AnalogInput armPot;

    double liftPower = 0;
    boolean holdLift = false;
    double intakePower = 0;


    double MAX_SPEED = 1.0;
    double armVolatge = 0;

    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
    private static final double MID_SERVO       =  0.5 ;

    potentiometer pot = new potentiometer();

    double armDegree = 0;
    double armTarget = 200;

    double armPower;
    double armExtendPower = 0;
    int armTargetPosition = 12;
    int armCurrentPosition = 12;

    double lastError = 0;
    double currentError = 0;
    double dError = 0;

    double ierror = 0;

    //PID Constants

    double ARMPID_KP = 0.01;
    double ARMPID_KD = 0;
    double ARMPID_KI = 0;

    double ARMEXTENDPID_KP = 0;

    int elbowTarget = 0;
    int elbowCurrent =0;
    int elbowError = 0;
    double elbowKp = .01;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        Lf = hardwareMap.get(DcMotor.class, "Lf");
        Lb = hardwareMap.get(DcMotor.class, "Lb");
        Rf = hardwareMap.get(DcMotor.class, "Rf");
        Rb = hardwareMap.get(DcMotor.class, "Rb");
        Li = hardwareMap.get(DcMotor.class, "Li");
        Ra = hardwareMap.get(DcMotor.class, "Ra");
        La = hardwareMap.get(DcMotor.class, "La");
        El = hardwareMap.get(DcMotor.class, "El");
        L  = hardwareMap.get(CRServo.class, "L");
        R  = hardwareMap.get(CRServo.class, "R");
        Lw = hardwareMap.get(Servo.class, "Lw");
        Rw = hardwareMap.get(Servo.class, "Rw");
        armPot = hardwareMap.get(AnalogInput.class, "armPot");



        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        Rf.setDirection(DcMotorSimple.Direction.FORWARD);
        Rb.setDirection(DcMotorSimple.Direction.FORWARD);
        Li.setDirection(DcMotorSimple.Direction.FORWARD);
        Ra.setDirection(DcMotorSimple.Direction.REVERSE);
        La.setDirection(DcMotorSimple.Direction.FORWARD);
        El.setDirection(DcMotorSimple.Direction.REVERSE);
        L.setDirection(CRServo.Direction.REVERSE);
        R.setDirection(CRServo.Direction.FORWARD);

        Lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Li.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Ra.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        La.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        El.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        El.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        armVolatge = armPot.getVoltage();
        armDegree = pot.degree(armVolatge);

      double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;
        holonomic(Speed, Turn, Strafe, MAX_SPEED );





        intakePower = gamepad2.left_trigger;
        if (gamepad1.left_trigger == 0 )
        {
            intakePower = -gamepad2.right_trigger;
        }
        L.setPower(intakePower);
        R.setPower(intakePower);

        if (gamepad2.y){
            elbowTarget -= 10;
           //if (elbowTarget <= 0){
           //     elbowTarget = 0;
           // }
        }
        if (gamepad2.x){
            elbowTarget += 10;
            //if (elbowTarget >=590){
           //     elbowTarget = 590;
           // }
        }

        //SET Elbow Postion
        El.setTargetPosition(elbowTarget);
        elbowCurrent = El.getCurrentPosition();
        elbowError = elbowTarget - elbowCurrent;
        El.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        El.setPower(elbowError*elbowKp);


       /* if (gamepad2.x){
            intakePower = 0.75;
        }
        else if (gamepad2.y){
            intakePower = -0.75;
        }
        else{
            intakePower = 0;
        }

        In.setPower(intakePower);
        */

        if (gamepad2.a){
            liftPower = 1;
        }
        else if(gamepad2.b){
            liftPower = -1;
        }
        else{
            liftPower = 0;
        }

        Li.setPower(liftPower);


        if (gamepad2.right_bumper){
            armTarget -= 1;
            if (armTarget <=100){
                armTarget = 100;
            }
        }

        if (gamepad2.left_bumper){
            armTarget += 1;
            if (armTarget >=260){
                armTarget = 260;
            }
        }
        /*if (gamepad1.right_bumper){
            armTarget = 100;

        }

        if (gamepad1.left_bumper){
            armTarget = 200;

        }
        */
        arm(armTarget);

        /* if (gamepad1.left_bumper && holdLift == false){
            Li.setPower(-1);
        }
        else if (gamepad1.left_bumper == false && holdLift== false){
            Li.setPower(0);
        }

        if (gamepad1.right_bumper && holdLift == false) {
            Li.setPower(1);
        }
        else if(gamepad1.right_bumper == false && holdLift == false){
            Li.setPower(0);
        }

        if (gamepad1.a)
        {
            liftPower = .5;
            holdLift = true;

        }

        if (gamepad1.b){
           holdLift = false;
        } */

        if (gamepad1.x){
            MAX_SPEED = 0.75;
        }
        if (gamepad1.y){
            MAX_SPEED = 1;
        }
        if (gamepad1.dpad_up) {
            MAX_SPEED = MAX_SPEED + 0.005;
        }
        if (gamepad1.dpad_down){
            MAX_SPEED = MAX_SPEED - 0.005;
        }
        if (MAX_SPEED > 1){
            MAX_SPEED = 1;
        }
        if (MAX_SPEED < 0.3){
            MAX_SPEED = 0.3;
        }


        if (gamepad2.dpad_up)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.dpad_down)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        Lw.setPosition(MID_SERVO + clawOffset);
        Rw.setPosition(MID_SERVO - clawOffset);

        // Send telemetry message to signify robot running;

       // telemetry.addData("Speed",  "%.2f", Speed);
        //telemetry.addData("Turn",  "%.2f", Turn);
       // telemetry.addData("Strafe", "%.2f", Strafe);
        telemetry.addData("MAX Speed", "%.2f", MAX_SPEED);
        telemetry.addData("armTarget",  "%.2f", armTarget);
        telemetry.addData("armDegree",  "%.2f", armDegree);


        telemetry.addData("Arm Power",  "%.2f", armPower);

        telemetry.addData("Intake Power",  "%.2f", intakePower);
        telemetry.addData("Elbow Error", elbowError);
        telemetry.addData("Elbow Power", elbowError*elbowKp);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //Li.setPower(liftPower);
    }

    public void arm(double target){
        double error = target - armDegree;
        currentError = error;

        //Calculate derivative

        dError = currentError - lastError;

        //Calculate integral

        if (error > 0){
            ierror = ierror + 0.1;
        }

        if (error < 0){
            ierror = ierror - 0.1;
        }

        if (error == 0){
            ierror = 0;
        }

        armPower = (ARMPID_KP * error) - (dError * ARMPID_KD) + (Range.clip(ierror*ARMPID_KI, -0.6, 0.6));
        La.setPower(Range.clip(armPower, -1, 1));
        Ra.setPower(Range.clip(armPower, -1, 1));

        currentError = lastError;

    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED) {

//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        Lf.setPower(scale((Speed + Turn - Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));

        if (Lb != null) {
            Lb.setPower(scale((Speed + Turn + Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        Rf.setPower(scale((Speed - Turn + Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (Rb != null) {
            Rb.setPower(scale((Speed - Turn - Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
    }
}
