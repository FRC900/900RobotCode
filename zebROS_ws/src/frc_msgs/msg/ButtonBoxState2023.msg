Header header

# diag mode or something else
bool lockingSwitchButton
bool lockingSwitchPress
bool lockingSwitchRelease

# Only red button, preempt all
bool redButton
bool redPress
bool redRelease

# top three buttons for setting elevator heights
bool topLeftConeButton
bool topLeftConePress
bool topLeftConeRelease

bool topMiddleConeButton
bool topMiddleConePress
bool topMiddleConeRelease

bool topRightCubeButton
bool topRightCubePress
bool topRightCubeRelease

# next three buttons for setting which node in the grid left to right
# should be from driver station perspective
bool gridSelectConeLeftButton
bool gridSelectConeLeftPress
bool gridSelectConeLeftRelease

bool gridSelectCubeButton
bool gridSelectCubePress
bool gridSelectCubeRelease

bool gridSelectConeRightButton
bool gridSelectConeRightPress
bool gridSelectConeRightRelease

# two stateful switches for low mid high and other for
# lowmidhigh ===
bool heightSelectSwitchUpButton
bool heightSelectSwitchUpPress
bool heightSelectSwitchUpRelease

bool heightSelectSwitchDownButton
bool heightSelectSwitchDownPress
bool heightSelectSwitchDownRelease
# lowmidhigh ===

# leftmidright ===
bool heightSelectSwitchLeftButton
bool heightSelectSwitchLeftPress
bool heightSelectSwitchLeftRelease

bool heightSelectSwitchRightButton
bool heightSelectSwitchRightPress
bool heightSelectSwitchRightRelease
# leftmidright ===

# left of the red button, right of the two switches, and 
bool centralYellowButton
bool centralYellowPress
bool centralYellowRelease

# just above green buttons
bool bottomLeftYellowButton
bool bottomLeftYellowPress
bool bottomLeftYellowRelease

# to the right of bottomleftyellow, extra button now
bool bottomRightWhiteButton
bool bottomRightWhitePress
bool bottomRightWhiteRelease

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