package org.firstinspires.ftc.teamcode.subsystems;

public class ToggleButton {
    public boolean toggleButtonPrevPressed;
    public boolean state;

    public ToggleButton(boolean initialState) {
        toggleButtonPrevPressed = false;
        state = initialState;
    }

    public void toggle(boolean toggleButton) {
        if (toggleButton & !toggleButtonPrevPressed) {
            if (state) {
                state = false;
            } else {
                state = true;
            }
            toggleButtonPrevPressed = true;
        } else if (!toggleButton & toggleButtonPrevPressed) {
            toggleButtonPrevPressed = false;
        }
    }

    public boolean state() {
        return state;
    }
}
