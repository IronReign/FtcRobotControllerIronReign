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

package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

//@Config(value = "00 This is a test")
@Disabled
@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class TestStuff extends OpMode
{

    static FtcDashboard dashboard = FtcDashboard.getInstance();
    // Declare OpMode members.
    double forward = 0;
    double strafe = 0;
    double turn = 0;
    double servoP=0;
    public static double driveS=1;
    StickyGamepad g1=null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private Servo s1=null;
    public static int test = 1;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        g1 = new StickyGamepad(gamepad1);
        rightFrontDrive  = hardwareMap.get(DcMotorEx.class, "rightF_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightB_drive");
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftF_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftB_drive");
        s1=hardwareMap.get(Servo.class, "random");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

//        rightFrontDrive.setMode(RunMode.);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        g1.update();

        if(g1.right_bumper && servoP<1){
            s1.setPosition(servoP+=.1);
        }
        if(g1.left_bumper && servoP>-1){
            s1.setPosition(servoP-=.1);
        }


        TelemetryPacket p = new TelemetryPacket();
        mecanumDrive();
        p.addLine("forward:"+forward+" strafe:"+strafe+" turn:"+turn);
        p.addLine("speed: "+driveS);
        p.addLine("Rotate position"+servoP);
        //p.addLine("DOES THIS WORK?");



        dashboard.sendTelemetryPacket(p);

         //dashboard.sendTelemetryPacket(p);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Speed",+ driveS);
    }

    public void mecanumDrive() {
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x;
        if(g1.right_bumper && driveS<1){
           driveS+=.1;
        }
        if(g1.left_bumper && driveS>0){
            driveS-=.1;
        }



        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI/4;
        double rightX = -turn;
        leftFrontDrive.setPower((r * Math.cos(robotAngle) - rightX)*driveS);
        rightFrontDrive.setPower((r * Math.sin(robotAngle) + rightX)*driveS);
       leftBackDrive.setPower((r * Math.sin(robotAngle) - rightX)*driveS);
       rightBackDrive.setPower((r * Math.cos(robotAngle) + rightX)*driveS);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



}
