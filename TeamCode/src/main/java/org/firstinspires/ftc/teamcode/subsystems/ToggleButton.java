package org.firstinspires.ftc.teamcode.subsystems;

public class ToggleButton {
    public boolean toggleButtonPrevPressed;
    public boolean state;
    public boolean newPress;

    public ToggleButton(boolean initialState) {
        toggleButtonPrevPressed = false;
        state = initialState;
        newPress = false;
    }

    public void toggle(boolean toggleButton) {
        if (toggleButton & !toggleButtonPrevPressed) {
            if (state) {
                state = false;
            } else {
                state = true;
            }
            newPress = true;
            toggleButtonPrevPressed = true;
        } else {
            newPress = false;
            if (!toggleButton & toggleButtonPrevPressed) {
                toggleButtonPrevPressed = false;
            }
        }
        }

    public boolean state() {
        return state;
    }

    public boolean newPress() {
        return newPress;
    }
}
