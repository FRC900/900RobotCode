Header header
uint8 CUBE=0
uint8 VERTICAL_CONE=1
uint8 BASE_TOWARDS_US_CONE=2 
uint8 BASE_AWAY_US_CONE=3
uint8 NONE=4 # for sure no game piece
uint8 UNKNOWN=5 # we have a game piece, but not sure what it is right now
uint8 game_piece

float64 offset_from_center # will be inf if terabees are 0 or inf