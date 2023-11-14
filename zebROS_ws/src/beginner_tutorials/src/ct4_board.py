#!/usr/bin/env python3

import rospy
from beginner_tutorials.msg import CT4State
from std_msgs.msg import UInt8

#### Copy pasted code

STEPS = [(-1, -1),(-1, 0),(-1, 1),(0, -1),(0, 1),(1, -1),(1, 0),(1, 1)]

def add_piece(board, column, player):
    """Adds a piece onto the board using the column of the piece"""
    not_filled = False
    for row_num in [5, 4, 3, 2, 1, 0]:
        if board[row_num][column] == 0:
            board[row_num][column] = player
            not_filled = True
            break
    return (board, not_filled)

def swap_player(player):
    """Swaps whose turn it is"""
    if player == 1:
        return 2
    return 1

def is_valid(row, col):
    """Returns true for valid board coordinates"""
    if row in range(6) and col in range(7):
        return True
    return False

def check_win_piece(board, row, col):
    """Checks if a certain piece is part of a connect four"""
    player = board[row][col]
    for step in STEPS:
        check_row = row
        check_col = col
        length = 1

        while length < 4:
            check_row += step[0]
            check_col += step[1]

            if not is_valid(check_row, check_col):
                break
            if board[check_row][check_col] != player:
                break

            length += 1

        if length == 4:
            return True

    return False

def check_win_piece_player(board, row, col, player):
    """Checks if a certain piece is part of a certain player's connect four"""
    player_found = board[row][col]
    if player_found != player:
        return False
    for step in STEPS:
        check_row = row
        check_col = col
        length = 1

        while length < 4:
            check_row += step[0]
            check_col += step[1]

            if not is_valid(check_row, check_col):
                break
            if board[check_row][check_col] != player:
                break

            length += 1

        if length == 4:
            return True

    return False

def check_win(board):
    """Checks the entire board for a win by either player"""
    for row in range(6):
        for col in range(7):
            if board[row][col] != 0:
                if check_win_piece(board, row, col):
                    return True
    return False

def check_win_player(board, player):
    """Checks the entire board for a win by a certain player"""
    for row in range(6):
        for col in range(7):
            if board[row][col] != 0:
                if check_win_piece_player(board, row, col, player):
                    return True
    return False

def check_draw(board):
    """Checks if the board is completely filled and the game is drawn"""
    for row in board:
        for cell in row:
            if cell == 0:
                return False
    return True

def check_for_ends(board):
    if check_draw(board):
        return 4
    if not check_win(board):
        return 1
    if check_win_player(board, 1):
        return 2
    return 3

#### ROSsy stuff begins here
# Reports:
# 0 - invalid
# 1 - success
# 2 - P1 win
# 3 - P2 win
# 4 - draw

def callback(data):
    global board
    global player
    (board, success) = add_piece(board, data.data - 1, player)
    if success:
        player = swap_player(player)
        report = check_for_ends(board)
    else:
        report = 0
    if report in [2, 3, 4]:
        player = 1
    
    publish(board, player, report)

    if report in [2, 3, 4]:
        board = [[0 for i in range(7)] for i in range(6)]
        
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber('/elias/ct4/moves', UInt8, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

pub = rospy.Publisher('/elias/ct4/board', CT4State, queue_size=1)
def publish(board, player, report):

    state = CT4State()
    state.row0 = board[0]
    state.row1 = board[1]
    state.row2 = board[2]
    state.row3 = board[3]
    state.row4 = board[4]
    state.row5 = board[5]
    state.player = player
    state.report = report
    pub.publish(state)

rospy.init_node('ct4_board_node', anonymous=True)

board = [[0 for i in range(7)] for i in range(6)]
player = 1
publish(board, 1, 1)
listener()