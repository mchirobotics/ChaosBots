/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.robotcore.util.Range.scale;
import static java.lang.Math.abs;


@TeleOp(name="TeleOp", group="TeleOp")

public class Techi_Driver_TeleOP extends OpMode {

    /* Declare OpMode members. */
    Techi_Hardware robot       = new Techi_Hardware();

    double MAX_SPEED = .5;

    int POSITION = 0;

//PID Constants


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



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



        //POSITION = 1025;
        //robot.liftarm.setPosition(.945);
        robot.grab.setPosition(.4);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if(gamepad1.left_stick_button){
            MAX_SPEED = 1;
        }
        if(gamepad1.right_stick_button){
            MAX_SPEED = .5;
        }


        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;
        holonomic(Speed, Turn, Strafe, MAX_SPEED );
        telemetry.addData("MAX Speed", "%.2f", MAX_SPEED);


        //ARM CODE BEGINS
        if(gamepad1.y){
            POSITION = 1490                                                                                                                                                                                                                                       ;
            robot.liftarm.setPosition(.2588);
        }
        if(gamepad1.b){
            POSITION = 915;

            robot.liftarm.setPosition(.49);

        }
        if(gamepad1.a){
            POSITION = 0;
            robot.liftarm.setPosition(1); // .753 - .98
            robot.grab.setPosition(.4);
        }
        if(gamepad1.x){
            POSITION = 1225;
            robot.liftarm.setPosition(.98);
            robot.grab.setPosition(.4);
        }
        if(gamepad1.right_bumper){
            POSITION = 550;
            robot.liftarm.setPosition(.1855);
        }
        if(gamepad1.left_bumper){
            POSITION = 0;
            robot.liftarm.setPosition(.6283);
        }
        //ARM CODE ENDS
        if(gamepad1.right_trigger > 0){
            robot.grab.setPosition(.5);
        }
        if(gamepad1.left_trigger > 0){
            robot.grab.setPosition(.4);
        }

        if(robot.laserBack.getDistance(DistanceUnit.INCH) < 1){
            gamepad1.rumble(100);
        }

        //Manual Mode

        if(gamepad1.dpad_up)
        {
            POSITION +=100;
            if(POSITION > 2600){POSITION= 2600;}

        }
        if(gamepad1.dpad_down)
        {
            POSITION -=100;
            if(POSITION < 0){POSITION = 0;}

        }
        if(gamepad1.dpad_left)
        {
            robot.liftarm.setPosition(robot.liftarm.getPosition() + .02);
            if(robot.liftarm.getPosition() > 1){robot.liftarm.setPosition(1);}
        }
        if(gamepad1.dpad_right)
        {
            robot.liftarm.setPosition(robot.liftarm.getPosition() - .02);
            if(robot.liftarm.getPosition() < 0){robot.liftarm.setPosition(0);}
        }
        lift(POSITION);

        telemetry.addData("ARM POSITION", POSITION);
        telemetry.addData("CURRENT POSITION", robot.lift.getCurrentPosition());
        telemetry.addData("Lift Arm", robot.liftarm.getPosition());
        telemetry.addData("Laser Back", String.format("%.01f in", robot.laserBack.getDistance(DistanceUnit.INCH)));
        telemetry.addData("Laser Front", String.format("%.01f in", robot.laserFront.getDistance(DistanceUnit.INCH)));

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void lift(int position){
        robot.lift.setTargetPosition(position);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);
    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED) {

//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        robot.lF.setPower(scale((Speed + Turn - Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));

        if (robot.lB != null) {
            robot.lB.setPower(scale((Speed + Turn + Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        robot.rF.setPower(scale((Speed - Turn + Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (robot.rB != null) {
            robot.rB.setPower(scale((Speed - Turn - Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
    }
}



