package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private static OI mInstance;
    public Joystick drivePad;
    public Joystick toolPad;
    public GenericHID keyboard;

    // Create a single instance of the OI
    public static OI getInstance() {
        if (mInstance == null) {
            mInstance = new OI();
            return mInstance;
        } else
            return mInstance;
    }

    /**
     * Initializes the joystick objects
     */
    public OI() {
        drivePad = new Joystick(GamepadConstants.DRIVE_USB_PORT);
        toolPad = new Joystick(GamepadConstants.TOOL_USB_PORT);
        keyboard = new GenericHID(GamepadConstants.KEYBOARD_PORT);
    }

    public double stickMap(double input) {
        return Math.signum(input) * Math.abs(Math.pow(input, 2));
    }

    /**
     * 
     * @return Returns the mapped drive of the right x axis
     */

    public double getMappedDriveRightX() {
        return stickMap(getDriveRightX());

    }

    /**
     * 
     * @return Returns the mapped drive of the right y axis
     */

    public double getMappedDriveRightY() {
        return stickMap(getDriveRightY());
    }

    /**
     * 
     * @return Returns the mapped drive of left x axis
     */

    public double getMappedDriveLeftX() {
        return stickMap(getDriveLeftX());
    }

    /**
     * 
     * @return Returns the mapped drive of the left y axis
     */

    public double getMappedDriveLeftY() {
        return stickMap(getDriveLeftY());
    }

    /**
     * Used to return the drivePad's right joystick y-axis value
     *
     * @return Returns y-value from right joystick on the drivePad
     */
    public double getDriveRightY() {
        double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return -1 * joy;
    }

    /**
     * Used to return the toolPad's right joystick y-axis value
     *
     * @return Returns y-value from right joystick on the toolPad
     */
    public double getToolRightY() {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return -1 * joy;
    }

    /**
     * Used to return the drivePad's left joystick y-axis value
     *
     * @return Returns y-value from left joystick on the drivePad
     */
    public double getDriveLeftY() {
        double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return -1 * joy;
    }

    /**
     * Used to return the toolPad's left joystick y-axis value
     *
     * @return Returns y-value from left joystick on the toolPad
     */
    public double getToolLeftY() {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return -1 * joy;
    }

    /**
     * Used to return the drivePad's right joystick x-axis value
     *
     * @return Returns x-value from right joystick on the drivePad
     */
    public double getDriveRightX() {
        double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    /**
     * Used to return the toolPad's right joystick x-axis value
     *
     * @return Returns x-value from right joystick on the toolPad
     */
    public double getToolRightX() {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    /**
     * Used to return the drivePad's left joystick x-axis value
     *
     * @return Returns x-value from left joystick on the drivePad
     */
    public double getDriveLeftX() {
        double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_X);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    /**
     * Used to return the toolPad's left joystick x-axis value
     *
     * @return Returns x-value from left joystick on the toolPad
     */
    public double getToolLeftX() {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_X);
        if (Math.abs(joy) < 0.05)
            return 0.0;
        else
            return joy;
    }

    // Drive DPAD up
    public boolean getDriveDPadUp() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_UP) {
            return true;
        } else {
            return false;
        }
    }

    // Drive DPAD right
    public boolean getDriveDPadRight() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_RIGHT) {
            return true;
        } else {
            return false;
        }
    }

    // Drive DPAD down
    public boolean getDriveDPadDown() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_DOWN) {
            return true;
        } else {
            return false;
        }
    }

    // Drive DPAD left
    public boolean getDriveDPadLeft() {
        if (drivePad.getPOV(0) == GamepadConstants.DPAD_LEFT) {
            return true;
        } else {
            return false;
        }
    }

    // Tool DPAD up
    public boolean getToolDPadUp() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_UP) {
            return true;
        } else {
            return false;
        }
    }

    // Tool DPAD right
    public boolean getToolDPadRight() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_RIGHT) {
            return true;
        } else {
            return false;
        }
    }

    // Tool DPAD down
    public boolean getToolDPadDown() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_DOWN) {
            return true;
        } else {
            return false;
        }
    }

    // Tool DPAD left
    public boolean getToolDPadLeft() {
        if (toolPad.getPOV(0) == GamepadConstants.DPAD_LEFT) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveRightTrigger() {
        return drivePad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightTrigger() {
        return toolPad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveLeftTrigger() {
        return drivePad.getRawButton(GamepadConstants.LEFT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftTrigger() {
        return toolPad.getRawButton(GamepadConstants.LEFT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveLeftBumper() {
        return drivePad.getRawButton(GamepadConstants.LEFT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftBumper() {
        return toolPad.getRawButton(GamepadConstants.LEFT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveRightBumper() {
        return drivePad.getRawButton(GamepadConstants.RIGHT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightBumper() {
        return toolPad.getRawButton(GamepadConstants.RIGHT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolXButton() {
        return toolPad.getRawButton(GamepadConstants.X_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolAButton() {
        return toolPad.getRawButton(GamepadConstants.A_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolBButton() {
        return toolPad.getRawButton(GamepadConstants.B_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolYButton() {
        return toolPad.getRawButton(GamepadConstants.Y_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveXButton() {
        return drivePad.getRawButton(GamepadConstants.X_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveAButton() {
        return drivePad.getRawButton(GamepadConstants.A_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveBButton() {
        return drivePad.getRawButton(GamepadConstants.B_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveYButton() {
        return drivePad.getRawButton(GamepadConstants.Y_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveBackButton() {
        return toolPad.getRawButton(GamepadConstants.BACK_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolStartButton() {
        return toolPad.getRawButton(GamepadConstants.START_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolBackButton() {
        return toolPad.getRawButton(GamepadConstants.BACK_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftButton() {
        return toolPad.getRawButton(GamepadConstants.LEFT_ANALOG_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightButton() {
        return toolPad.getRawButton(GamepadConstants.RIGHT_ANALOG_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveStartButton() {
        return drivePad.getRawButton(GamepadConstants.START_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardQ() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_Q);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardW() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_W);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardE() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_E);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardR() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_R);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardT() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_T);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardY() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_Y);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardU() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_U);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardA() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_A);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardS() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_S);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardD() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_D);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardF() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_F);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardG() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_G);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardH() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_H);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardJ() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_J);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardZ() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_Z);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardX() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_X);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardC() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_C);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardV() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_V);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardB() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_B);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardN() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_N);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getKeyboardM() {
        return keyboard.getRawButton(GamepadConstants.Keyboard_M);
    }

    public int getKeyboardButtonPressed() {
        for (int i = 1; i <= 27; i++) {
            if (keyboard.getRawButton(i)) {
                return i;
            }
        }
        return 0;
    }

}