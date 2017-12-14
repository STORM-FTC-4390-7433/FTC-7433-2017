package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ajit Singh on 10/6/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="SuperNova", group="TeleOp")
public class TeleOp extends OpMode {
    private DcMotor leftMotor = null;
    private DcMotor jewelMotor = null;
    private DcMotor pulleyMotor = null;
    private DcMotor rightMotor = null;
    private Servo leftArmServo = null;
    private Servo rightArmServo = null;


    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        pulleyMotor = hardwareMap.dcMotor.get("pulleyMotor");
        jewelMotor = hardwareMap.dcMotor.get("jewelMotor");
        leftArmServo = hardwareMap.servo.get("leftArmServo");
        rightArmServo = hardwareMap.servo.get("rightArmServo");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop() {
        float throttleLeft = gamepad1.left_stick_y;
        float throttleRight = gamepad1.right_stick_y;

        if (gamepad2.b)
            jewelMotor.setPower(.2);
        else if (gamepad2.x)
            jewelMotor.setPower(-.2);
        else
            jewelMotor.setPower(0);

        if (gamepad2.y)
            pulleyMotor.setPower(.7);
        else if (gamepad2.a)
            pulleyMotor.setPower(-.7);
        else
            pulleyMotor.setPower(0);

        if (gamepad2.left_bumper)
            leftArmServo.setPosition(0);
        //else if (gamepad2.left_trigger < 0)
            //leftArmServo.setPosition(60);
        else
            leftArmServo.setPosition(.3);

        if (gamepad2.right_bumper)
            rightArmServo.setPosition(1);
       // else if (gamepad2.right_trigger > 0)
            //rightArmServo.setPosition(170);
        else
            rightArmServo.setPosition(.7);


        throttleRight = Range.clip(throttleRight, -1, 1);
        throttleLeft = Range.clip(throttleLeft, -1, 1);

        throttleRight = (float)scaleInput(throttleRight);
        throttleLeft =  (float)scaleInput(throttleLeft);

        leftMotor.setPower(-throttleLeft);
        rightMotor.setPower(-throttleRight);
    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0
                ;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}

