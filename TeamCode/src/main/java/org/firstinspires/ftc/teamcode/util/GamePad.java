package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

@SuppressWarnings("unused")
public class GamePad {

    private final Gamepad gp;
    private int a, b, x, y, dpadUp, dpadDown, dpadLeft, dpadRight, leftBumper, rightBumper,
            leftButton, rightButton, start, back, leftTrigger, rightTrigger;
    private double lx, ly, rx, ry, lt, rt;

    public GamePad(Gamepad gp) {
        this.gp = gp;
    }

    // Call this at the end of the loop:
    public void update() {
        if (gp.a) a++;
        else a = 0;
        if (gp.b) b++;
        else b = 0;
        if (gp.x) x++;
        else x = 0;
        if (gp.y) y++;
        else y = 0;
        if (gp.dpad_up) dpadUp++;
        else dpadUp = 0;
        if (gp.dpad_down) dpadDown++;
        else dpadDown = 0;
        if (gp.dpad_left) dpadLeft++;
        else dpadLeft = 0;
        if (gp.dpad_right) dpadRight++;
        else dpadRight = 0;
        if (gp.left_bumper) leftBumper++;
        else leftBumper = 0;
        if (gp.right_bumper) rightBumper++;
        else rightBumper = 0;
        if (gp.left_stick_button) leftButton++;
        else leftButton = 0;
        if (gp.right_stick_button) rightButton++;
        else rightButton = 0;
        if (gp.start) start++;
        else start = 0;
        if (gp.back) back++;
        else back = 0;
        if (gp.left_trigger > 0) leftTrigger++;
        else leftTrigger = 0;
        if (gp.right_trigger > 0) rightTrigger++;
        else rightTrigger = 0;

        lx = gp.left_stick_x;
        ly = gp.left_stick_y;
        rx = gp.right_stick_x;
        ry = gp.right_stick_y;
        lt = gp.left_trigger;
        rt = gp.right_trigger;
    }

    public boolean a() { return a > 0; }

    public boolean b() { return b > 0; }

    public boolean x() { return x > 0; }

    public boolean y() { return y > 0; }

    public boolean dpadUp() { return dpadUp > 0; }

    public boolean dpadDown() { return dpadDown > 0; }

    public boolean dpadLeft() { return dpadLeft > 0; }

    public boolean dpadRight() { return dpadRight > 0; }

    public boolean leftBumper() { return leftBumper > 0; }

    public boolean rightBumper() { return rightBumper > 0; }

    public boolean leftStickButton() { return leftButton > 0; }

    public boolean rightStickButton() { return rightButton > 0; }

    public boolean start() { return start > 0; }

    public boolean back() { return back > 0; }

    public double leftStickX() { return lx; }

    public double leftStickY() { return ly; }

    public double rightStickX() { return rx; }

    public double rightStickY() { return ry; }

    public double leftTrigger() { return lt; }

    public double rightTrigger() { return rt; }


    public boolean onceA() { return a == 1; }

    public boolean onceB() { return b == 1; }

    public boolean onceX() { return x == 1; }

    public boolean onceY() { return y == 1; }

    public boolean onceDpadUp() { return dpadUp == 1; }

    public boolean onceDpadDown() { return dpadDown == 1; }

    public boolean onceDpadLeft() { return dpadLeft == 1; }

    public boolean onceDpadRight() { return dpadRight == 1; }

    public boolean onceLeftBumper() { return leftBumper == 1; }

    public boolean onceRightBumper() { return rightBumper == 1; }

    public boolean onceLeftStickButton() { return leftButton == 1; }

    public boolean onceRightStickButton() { return rightButton == 1; }

    public boolean onceBack() { return back == 1; }

    public boolean onceStart() { return start == 1; }

    public boolean onceLeftTrigger() { return leftTrigger == 1; }

    public boolean onceRightTrigger() { return rightTrigger == 1; }


    public boolean longA() { return a >= 10; }

    public boolean longB() { return b >= 10; }

    public boolean longX() { return x >= 10; }

    public boolean longY() { return y >= 10; }

    public boolean longDpadUp() { return dpadUp >= 10; }

    public boolean longDpadDown() { return dpadDown >= 10; }

    public boolean longDpadLeft() { return dpadLeft >= 10; }

    public boolean longDpadRight() { return dpadRight >= 10; }

    public boolean longLeftBumper() { return leftBumper >= 10; }

    public boolean longRightBumper() { return rightBumper >= 10; }

    public boolean longLeftStickButton() { return leftButton >= 10; }

    public boolean longRightStickButton() { return rightButton >= 10; }
}
