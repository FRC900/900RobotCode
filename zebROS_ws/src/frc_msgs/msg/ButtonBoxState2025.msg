Header header

# safe vs not safe
bool notSafeModeLockingSwitchButton
bool notSafeModeLockingSwitchPress
bool notSafeModeLockingSwitchRelease

# auto mode
uint8 auto_mode

# zero button
bool zeroButton
bool zeroPress
bool zeroRelease

# Only red button, preempt all
bool redButton
bool redPress
bool redRelease

# backup buttons
bool outtakeButton
bool outtakePress
bool outtakeRelease

bool backupButton2Button
bool backupButton2Press
bool backupButton2Release

# elevator to L2
bool elevatorL2Button
bool elevatorL2Press
bool elevatorL2Release

# elevator retract
bool elevatorRetractButton
bool elevatorRetractPress
bool elevatorRetractRelease

# manual intake
bool intakeButton
bool intakePress
bool intakeRelease

# stateful switch
# ===
bool leftSwitchUpButton
bool leftSwitchUpPress
bool leftSwitchUpRelease

bool leftSwitchDownButton
bool leftSwitchDownPress
bool leftSwitchDownRelease
# ===

# stateful switch
# speed ===
bool rightSwitchUpButton
bool rightSwitchUpPress
bool rightSwitchUpRelease

bool rightSwitchDownButton
bool rightSwitchDownPress
bool rightSwitchDownRelease
# speed ===

# green buttons ====
bool leftGreenButton
bool leftGreenPress
bool leftGreenRelease

bool rightGreenButton
bool rightGreenPress
bool rightGreenRelease

bool topGreenButton
bool topGreenPress
bool topGreenRelease

bool bottomGreenButton
bool bottomGreenPress
bool bottomGreenRelease
# green buttons ====