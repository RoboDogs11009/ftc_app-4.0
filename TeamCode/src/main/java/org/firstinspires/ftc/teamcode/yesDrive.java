package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
   // private DcMotor Li = null;
    private DcMotor In = null;
    private DcMotor Ra = null;
    private DcMotor La = null;

    double liftPower=0;
    boolean holdLift = false;

//hi alan is cool





    double MAX_SPEED = 1.0;

    double rollerPower = 0;

    double slowDrive = 0.75;
    double SDinc = 0.005;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        Lf  = hardwareMap.get(DcMotor.class, "Lf");
        Lb = hardwareMap.get(DcMotor.class, "Lb");
        Rf  = hardwareMap.get(DcMotor.class, "Rf");
        Rb = hardwareMap.get(DcMotor.class, "Rb");
       // Li = hardwareMap.get(DcMotor.class, "Li");
        In = hardwareMap.get(DcMotor.class, "In");
        Ra = hardwareMap.get(DcMotor.class, "Ra");
        La = hardwareMap.get(DcMotor.class, "La");


        Lf.setDirection(DcMotorSimple.Direction.REVERSE);
        Lb.setDirection(DcMotorSimple.Direction.REVERSE);
        Rf.setDirection(DcMotorSimple.Direction.FORWARD);
        Rb.setDirection(DcMotorSimple.Direction.FORWARD);
      //Li.setDirection(DcMotorSimple.Direction.FORWARD);
        In.setDirection(DcMotorSimple.Direction.FORWARD);
        Ra.setDirection(DcMotorSimple.Direction.FORWARD);
        La.setDirection(DcMotorSimple.Direction.REVERSE);

        Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //Li.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        In.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Ra.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        La.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


      double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;
        holonomic(Speed, Turn, Strafe, MAX_SPEED );
        double armPower = gamepad2.left_stick_y;
        double intakePower = gamepad2.right_stick_y;

        In.setPower(intakePower);
        La.setPower(armPower);
        Ra.setPower(armPower);

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


        // Send telemetry message to signify robot running;

        telemetry.addData("Speed",  "%.2f", Speed);
        telemetry.addData("Turn",  "%.2f", Turn);
        telemetry.addData("Strafe", "%.2f", Strafe);
        telemetry.addData("MAX Speed", "%.2f", MAX_SPEED);
        telemetry.addData("Roller Power",  "%.2f", rollerPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //Li.setPower(liftPower);
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
