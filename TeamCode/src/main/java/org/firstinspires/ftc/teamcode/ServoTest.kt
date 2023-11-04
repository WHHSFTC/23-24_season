package org.thenuts.powerplay.opmode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

//ServoTest
@TeleOp(name = "ServoTest", group = "Test")
class ServoTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val servos = hardwareMap.getAll(Servo::class.java)
        waitForStart()
        val tester1: Tester
        val tester2: Tester
        try {
            tester1 = Tester(gamepad1, servos, "d1")
            tester2 = Tester(gamepad2, servos, "d2")
        } catch (ex: Exception) {
            return
        }
        val thread1 = Thread {
            try {
                tester1.testServos()
            } catch (ex: InterruptedException) {
            }
        }
        val thread2 = Thread {
            try {
                tester2.testServos()
            } catch (ex: InterruptedException) {
            }
        }
        thread1.start()
        thread2.start()
        while (opModeIsActive()) {
            tester1.log(telemetry)
            tester2.log(telemetry)
            telemetry.update()
            Thread.sleep(500)
        }
    }

    internal inner class Tester(
        private val gamepad: Gamepad,
        private val servos: List<Servo>,
        private val name: String
    ) {
        var i = 0
        var servo: Servo? = null
        var stick = false
        var prevInc = false
        var prevDec = false
        var prevStick = false
        var prevX = false
        var offset = 0.0
        var precision = 0.05
        var a = 0.0
        var b = 0.5
        var y = 1.0
        fun log(telemetry: Telemetry) {
            telemetry.addData("Stick$name", stick)
            telemetry.addData("Offset$name", offset)
            telemetry.addData("Precision$name", precision)
            //telemetry.addData("Controller" + name, servo.getController().getDeviceName());
//            telemetry.addData("Port" + name, servo.getPortNumber());
            telemetry.addData("Names$name", hardwareMap.getNamesOf(servo))
            telemetry.addData("Position$name", servo!!.position)
            telemetry.addData("a$name", a)
            telemetry.addData("b$name", b)
            telemetry.addData("y$name", y)
            telemetry.addData("enabled$name", (servo as ServoImplEx?)!!.isPwmEnabled)
        }

        @Throws(InterruptedException::class)
        fun testServos() {
            while (opModeIsActive()) {
                servo = servos[i]
                if (gamepad.right_bumper && !prevInc && i < servos.size - 1) i++
                prevInc = gamepad.right_bumper
                if (gamepad.left_bumper && !prevDec && i > 0) i--
                prevDec = gamepad.left_bumper
                if (gamepad.left_trigger > 0.5) {
                    if (gamepad.a) a = servo!!.position
                    if (gamepad.b) b = servo!!.position
                    if (gamepad.y) y = servo!!.position
                } else if (gamepad.right_trigger > 0.5) {
                    if (gamepad.a) servo!!.position = a
                    if (gamepad.b) servo!!.position = b
                    if (gamepad.y) servo!!.position = y
                } else {
//                    if (gamepad.x) offset = servo.getPosition();
//                    if (gamepad.b && !prevStick) stick = !stick;
//                    prevStick = gamepad.b;
                }
                if (gamepad.x && !prevX) {
                    if ((servo as ServoImplEx?)!!.isPwmEnabled) {
                        (servo as ServoImplEx?)!!.setPwmDisable()
                    } else {
                        (servo as ServoImplEx?)!!.setPwmEnable()
                    }
                }
                if (gamepad.dpad_left) precision = 0.05
                if (gamepad.dpad_right) precision = 0.01
                //                if (stick) {
//                    servo.setPosition(gamepad.left_stick_y * precision + offset);
//                } else {
                if (gamepad.dpad_up && servo!!.position <= 1 - precision) {
                    servo!!.position = servo!!.position + precision
                    Thread.sleep(200)
                }
                if (gamepad.dpad_down && servo!!.position >= precision) {
                    servo!!.position = servo!!.position - precision
                    Thread.sleep(200)
                }
                //                }
            }
        }
    }
}