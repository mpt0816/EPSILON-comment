# @file terminal_server.py
# @author HKUST Aerial Robotics Group
# @brief terminal server for testing
# @version 0.1
# @date 2019-02
# @copyright Copyright (c) 2019

# sys
import time
import sys
import rospy
import shutil
import random
from math import *
from functools import reduce
# pygame, 开源游戏开发库，主要用于2D游戏。
# 提供了对音频、图形、输入和窗口的访问。
import pygame as pg
from pygame.locals import *
from pygame.math import Vector2
# msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from vehicle_msgs.msg import ArenaInfoDynamic
from vehicle_msgs.msg import ArenaInfoStatic
from vehicle_msgs.msg import State
from vehicle_msgs.msg import ControlSignal

ego_id = 0
agent_id = 0
width = 800
height = 600
scale = 4.0
# 创建窗口
screen = pg.display.set_mode((width, height))
# 用于管理和渲染精灵对象的容器
all_sprites = pg.sprite.Group()
joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
vehicles = {}
recorded_ids = []
lane_pts = []
# 主车的位置
center_3dof = (0.0, 0.0, 0.0)
state_seq = []
has_arena_info_dynamic = False

# 转化到以主车为中心的坐标系上，并进行缩小
def project_world_to_image(point_3dof):
    x = point_3dof[0] - center_3dof[0]
    y = point_3dof[1] - center_3dof[1]
    angle = center_3dof[2]
    u = width/2 + scale * (x*sin(angle) - y * cos(angle))
    v = height/2 - scale * (x*cos(angle) + y * sin(angle))
    return (u, v)

# 显示主车的方向盘
class Wheel(pg.sprite.Sprite):
    def __init__(self, screen_rect):
        pg.sprite.Sprite.__init__(self)
        # 加载一个 PNG 格式的方向盘图像，并将其转换为 Pygame 可以处理的格式，以便后续在游戏窗口中使用
        self.original_image = pg.image.load(
            "steer_wheel.png").convert_alpha()
        # 将 PNG 图片缩小为原始的20%
        self.original_image = pg.transform.rotozoom(
            self.original_image, 0.0, 0.2)
        # self.original_image.set_colorkey((255, 255, 255))
        # self.original_image.
        self.image = self.original_image
        self.rect = self.image.get_rect()
        self.rect.center = (100, 100)

    def update(self):
        angle, acc = calc_current_steer_acc()
        # 弧度转化为角度
        angle = angle * 180 / pi
        # 根据控制转角旋转 方向盘 图片
        self.image = pg.transform.rotate(self.original_image, angle)
        x, y = self.rect.center
        self.rect = self.image.get_rect()  # Replace old rect with new rect.
        self.rect.center = (x, y)  # Put the new rect's center at old center.

# 显示车辆，用圆形表示
# 主车： 绿色
# 障碍物车辆： 蓝色
class Vehicle(pg.sprite.Sprite):
    def __init__(self, screen_rect, id, state_3dof):
        pg.sprite.Sprite.__init__(self)
        self.id = id
        self.state_3dof = state_3dof
        self.radius = 10
        self.image = pg.Surface((self.radius*2, self.radius*2), pg.SRCALPHA)
        if id == ego_id:
            pg.draw.circle(self.image, pg.Color(
                'darkolivegreen1'), (self.radius, self.radius), self.radius)
        else:
            pg.draw.circle(self.image, pg.Color(
                'dodgerblue1'), (self.radius, self.radius), self.radius)

        self.rect = self.image.get_rect(
            center=project_world_to_image(state_3dof))
        self.screen_rect = screen_rect

    def update(self):
        latest_state_3dof = (vehicles[self.id].vec_position.x,
                             vehicles[self.id].vec_position.y, vehicles[self.id].angle)
        self.rect.center = project_world_to_image(latest_state_3dof)


def calc_current_steer_acc():
    if len(state_seq) < 2:
        return 0.0, 0.0
    steer_list = []
    for i in range(len(state_seq)-1):
        state1 = state_seq[i]
        # 2.85 是轴距
        steer = atan(state1.curvature * 2.85)
        steer_list.append(steer)
    ave_steer = reduce(lambda x, y: x + y, steer_list) / len(steer_list)
    rotated_angle = ave_steer * 1.25 * (360.0/45.0)
    return rotated_angle, state_seq[-1].acceleration


def plot_lanes_on_screen():
    lanepts_plot = list(lane_pts)
    for i in range(len(lanepts_plot)):
        pg.draw.lines(screen, pg.Color('deeppink'), False, lanepts_plot[i])

# 显示主车速度和加速度
def plot_speed_on_screen():
    speed = 0.0
    acc = 0.0
    if len(state_seq) > 1:
        # 转化单位为 km/h
        speed = state_seq[-1].velocity * 3.6
        acc = state_seq[-1].acceleration
    font_obj = pg.font.Font('freesansbold.ttf', 20)
    text_surface_obj = font_obj.render(
        'vel: {:.2f} km/h '.format(speed), True, (0, 0, 0))
    text_rect_obj = text_surface_obj.get_rect()
    text_rect_obj.center = (110, 180)
    screen.blit(text_surface_obj, text_rect_obj)
    text_surface_obj = font_obj.render(
        'acc: {:.2f} m/s^2'.format(acc), True, (0, 0, 0))
    text_rect_obj = text_surface_obj.get_rect()
    text_rect_obj.center = (110, 200)
    screen.blit(text_surface_obj, text_rect_obj)

# 在屏幕上绘制车辆的标识符（文本）
def plot_ids_on_screen():
    for idx in recorded_ids:
        state_3dof = (vehicles[idx].vec_position.x,
                      vehicles[idx].vec_position.y, vehicles[idx].angle)
        # 使用字体 'freesansbold.ttf' 创建字体对象，字号为 16
        font_obj = pg.font.Font('freesansbold.ttf', 16)
        # 使用字体对象创建文本表面对象，渲染车辆的标识符，并设置文本颜色为绿色
        text_surface_obj = font_obj.render(
            '{}'.format(idx), True, (0, 255, 0))
        # 获取文本表面对象的矩形区域
        text_rect_obj = text_surface_obj.get_rect()
        u, v = project_world_to_image(state_3dof)
        # 将文本矩形区域的中心设置为车辆投影位置的偏移（向左偏移20个像素），用于在车辆附近显示标识符
        text_rect_obj.center = (u-20, v)
        # 将文本表面对象绘制到屏幕上
        screen.blit(text_surface_obj, text_rect_obj)

# 用一条黑色的线段表示车辆航向
def plot_orientations_on_screen():
    for idx in recorded_ids:
        state_3dof = (vehicles[idx].vec_position.x,
                      vehicles[idx].vec_position.y, vehicles[idx].angle)
        u, v = project_world_to_image(state_3dof)
        angle_diff = vehicles[idx].angle - center_3dof[2]
        pt1 = (u, v)
        pt2 = (u - 10 * sin(angle_diff), v - 10 * cos(angle_diff))
        pg.draw.line(screen, pg.Color('black'), pt1, pt2, 3)

# 绘制车辆被鼠标可选中区域
def plot_selected_rect_on_screen():
    if agent_id in recorded_ids:
        agent_state = (vehicles[agent_id].vec_position.x,
                       vehicles[agent_id].vec_position.y, vehicles[agent_id].angle)
        u, v = project_world_to_image(agent_state)
        pg.draw.rect(screen, pg.Color('aquamarine3'), (u-10, v-10, 20, 20), 3)


def process_arena_info_dynamic(data):
    for v in data.vehicle_set.vehicles:
        vehicles[v.id.data] = v.state
        if v.id.data not in recorded_ids:
            recorded_ids.append(v.id.data)
            screen_rect = screen.get_rect()
            all_sprites.add(Vehicle(screen_rect, v.id.data, (v.state.vec_position.x,
                                                             v.state.vec_position.y, v.state.angle)))
    global center_3dof, has_arena_info_dynamic
    # 主车的位置
    center_3dof = (vehicles[ego_id].vec_position.x,
                   vehicles[ego_id].vec_position.y, vehicles[ego_id].angle)
    has_arena_info_dynamic = True


def process_arena_info_static(data):
    global lane_pts
    if has_arena_info_dynamic:
        visible_range = 150.0
        del lane_pts[:]
        for lane in data.lane_net.lanes:
            points = []
            # has_first_pt = False
            # 转化到车体（主车）坐标下
            for pt in lane.points:
                if abs(pt.x - center_3dof[0]) < visible_range and abs(pt.y - center_3dof[1]) < visible_range:
                    # has_first_pt = True
                    points.append(project_world_to_image((pt.x, pt.y, 0.0)))
                # else:
                #     if has_first_pt:
                #         break
            if len(points) > 2:
                lane_pts.append(points)


def process_control_signal(data):
    global state_seq
    state_seq.append(data.state)
    if len(state_seq) > 10:
        state_seq.pop(0)


def update_visualization():
    if has_arena_info_dynamic:
        all_sprites.update()
        screen.fill(pg.Color('cornsilk2'))
        all_sprites.draw(screen)
        plot_lanes_on_screen()
        plot_ids_on_screen()
        plot_selected_rect_on_screen()
        plot_speed_on_screen()
        plot_orientations_on_screen()


def print_over_same_line(text):
    terminal_width = shutil.get_terminal_size((80, 20)).columns
    empty_space = max(0, terminal_width - len(text))
    sys.stdout.write('\r' + text + empty_space * ' ')
    sys.stdout.flush()


def init_joy(frame_id):
    joy = Joy()
    joy.header.frame_id = frame_id
    joy.header.stamp = rospy.Time.now()
    for i in range(8):
        joy.axes.append(0.0)
    for i in range(11):
        joy.buttons.append(0)
    return joy

# 处理键盘和鼠标事件
def handle_keyboard_event():
    global agent_id
    for event in pg.event.get():
        # 处理鼠标按钮弹起事
        if event.type == pg.MOUSEBUTTONUP:
            pos = pg.mouse.get_pos()
            # 通过碰撞检测找到被点击的精灵对象
            clicked_sprites = [
                s for s in all_sprites if s.rect.collidepoint(pos)]
            # 如果找到被点击的精灵对象，并且该精灵对象具有 'id' 属性
            if len(clicked_sprites) > 0:
                if hasattr(clicked_sprites[0], 'id'):
                    agent_id = clicked_sprites[0].id
                    print('update agent id to ', agent_id)
        # 处理键盘按下事件
        if event.type == KEYDOWN:
            joy = init_joy("{}".format(agent_id))
            if event.key == pg.K_w:
                msg = 'Agent {}: Speed up'.format(agent_id)
                print(msg)
                joy.buttons[3] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_s:
                msg = 'Agent {}: Brake'.format(agent_id)
                print(msg)
                joy.buttons[0] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_a:
                msg = 'Agent {}: Lane change left'.format(agent_id)
                print(msg)
                joy.buttons[2] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_d:
                msg = 'Agent {}: Lane change right'.format(agent_id)
                print(msg)
                joy.buttons[1] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_q:
                ## 切换 左侧是否可以换道的状态
                msg = 'Agent {}: Toggle left lc feasible state'.format(
                    agent_id)
                print(msg)
                joy.buttons[4] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_e:
                ## 切换 右侧是否可以换道的状态
                msg = 'Agent {}: Toggle right lc feasible state'.format(
                    agent_id)
                print(msg)
                joy.buttons[5] = 1
                joy_pub.publish(joy)
            elif event.key == pg.K_r:
                ## 切换 自动驾驶状态
                msg = 'Agent {}: Toggle autonomous mode'.format(agent_id)
                print(msg)
                joy.buttons[6] = 1
                joy_pub.publish(joy)


def main():
    # initialize pygame to get keyboard event
    pg.init()
    screen.fill(pg.Color('cornsilk3'))
    screen_rect = screen.get_rect()
    # 画出方向盘
    all_sprites.add(Wheel(screen_rect))
    all_sprites.draw(screen)
    pg.display.set_caption('Ultimate Vehicle Planning')
    pg.display.update()
    # initialize ros publisher
    # 订阅 动态障碍物 topic
    rospy.Subscriber("/arena_info_dynamic", ArenaInfoDynamic,
                     process_arena_info_dynamic)
    # 订阅 静态障碍物 topic
    rospy.Subscriber("/arena_info_static", ArenaInfoStatic,
                     process_arena_info_static)
    # 订阅 控制指令 topic
    rospy.Subscriber("/ctrl/agent_0", ControlSignal, process_control_signal)
    rospy.init_node('key2joy')
    rate = rospy.Rate(50)

    print('Terminal server initialized.')
    while not rospy.is_shutdown():
        handle_keyboard_event()
        update_visualization()
        pg.display.update()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
