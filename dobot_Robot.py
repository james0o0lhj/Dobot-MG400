import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np

# 全局变量(当前坐标)
current_actual = None

def connect_robot():
    try:
        ip = "192.168.1.6"
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004
        print("connecting...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print(">.<connected>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(failed:(")
        raise e

def run_point(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])

def get_feed(feed: DobotApi):
    global current_actual
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0

        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':

            # Refresh Properties
            current_actual = a["tool_vector_actual"][0]
            #print("tool_vector_actual:", current_actual)

        sleep(0.001)

def wait_arrive(point_list):
    global current_actual
    while True:
        is_arrive = True

        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False

            if is_arrive:
                return

        sleep(0.001)

def dobot_init(move: DobotApiMove,dashboard:DobotApi):
    dashboard.ResetRobot()
    point_init = [300, 0, 0, 0]
    run_point(move, point_init) 
    wait_arrive(point_init)
    print("dobot initial position",point_init)




if __name__ == '__main__':
    dashboard, move, feed = connect_robot()

    # print("EnableRobot...")
    #dashboard.EnableRobot()
    print("Enabled:)")

    #dashboard.DisableRobot()
    dashboard.ClearError()
    dashboard.DisableRobot()


    

    # feed_thread = threading.Thread(target=get_feed, args=(feed,))
    # feed_thread.setDaemon(True)
    # feed_thread.start()
    # print("running...")


    # point_a = [20, 280, -60, 0]
    # point_b = [160, 260, -30, 0]  
    # run_point(move, point_a)

    # run_point(move, point_b)
    # wait_arrive(point_b)
    #dobot_init(move,dashboard)

    