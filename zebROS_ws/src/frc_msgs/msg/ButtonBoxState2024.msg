Header header

# safe vs not safe
bool lockingSwitchButton
bool lockingSwitchPress
bool lockingSwitchRelease

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
bool backupButton1Button
bool backupButton1Press
bool backupButton1Release

bool backupButton2Button
bool backupButton2Press
bool backupButton2Release

# trap
bool trapButton
bool trapPress
bool trapRelease

# climb
bool climbButton
bool climbPress
bool climbRelease

# momentary switch for speed
# speed ===
bool speedSwitchUpButton
bool speedSwitchUpPress
bool speedSwitchUpRelease

bool speedSwitchDownButton
bool speedSwitchDownPress
bool speedSwitchDownRelease
# speed ===

# stateful switch for shooter vs arm (if arm mode, does trap climb)
# speed ===
bool shooterArmUpButton
bool shooterArmUpPress
bool shooterArmUpRelease

bool shooterArmDownButton
bool shooterArmDownPress
bool shooterArmDownRelease
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