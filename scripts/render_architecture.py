#!/usr/bin/env python3
"""Generate the README cover and architecture diagrams with the standard library.

Coordinates are intentional: retain the reference's top-down, rounded blue boxes,
grey connectors and grouped modes. Labels are SVG text, not raster screenshots.
Dashed lines describe launch/optional relationships; solid lines carry runtime data.
Run from any directory; --check never writes files.
"""
import argparse
from html import escape
from pathlib import Path
import math
import unicodedata
import xml.etree.ElementTree as ET

from generate_map import obstacles
from warehouse_route import STATIONS

ROOT = Path(__file__).resolve().parents[1]
FONT = 'Noto Sans CJK SC,Microsoft YaHei,Droid Sans Fallback,DejaVu Sans,sans-serif'
BLUE, FILL, BORDER, LINE = '#00569b', '#e4f3ff', '#cfdae3', '#91999f'


def units(text):
    return sum(1 if unicodedata.east_asian_width(c) in 'WF' else .58 for c in text)


class Diagram:
    def __init__(self, width, height, title, subtitle):
        self.w, self.h = width, height
        self.groups, self.edges, self.nodes, self.labels = [], [], [], []
        self.title, self.subtitle = title, subtitle

    def text(self, x, y, text, size=22, bold=False, color=BLUE, anchor='middle'):
        return (f'<text x="{x:g}" y="{y:g}" font-size="{size:g}" '
                f'font-weight="{700 if bold else 400}" fill="{color}" '
                f'text-anchor="{anchor}">{escape(text)}</text>')

    def group(self, x, y, w, h, title):
        self.groups.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" '
                           'rx="14" fill="#f5f7f9" stroke="#d9dfe4" stroke-width="2"/>')
        self.labels.append(self.text(x+w/2, y+39, title, 25, True, '#3d5568'))

    def box(self, x, y, w, h, title, lines=(), small=False, optional=False):
        assert x >= 0 and y >= 0 and x+w <= self.w and y+h <= self.h
        titles = title.split('\n')
        lines = list(lines)
        leading = 32 if not small else 29
        total = len(titles)*leading + len(lines)*29 + (8 if lines else 0)
        assert total <= h-18, (title, h, total)
        self.nodes.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="25" '
                          f'fill="{FILL}" stroke="{BORDER}" stroke-width="2"'
                          + (' stroke-dasharray="8 6"' if optional else '') + '/>')
        baseline = y+(h-total)/2+24
        for line in titles:
            size = min(25 if not small else 23, (w-32)/max(1,units(line)))
            self.nodes.append(self.text(x+w/2, baseline, line, size, True))
            baseline += leading
        if lines:
            baseline += 8
        for line in lines:
            size = min(21, (w-32)/max(1,units(line)))
            self.nodes.append(self.text(x+w/2, baseline, line, size))
            baseline += 29
        return x, y, w, h

    def pill(self, x, y, text, size=21, width=None):
        w = width or math.ceil(units(text)*size+34)
        self.labels.append(f'<rect x="{x-w/2:g}" y="{y-23:g}" width="{w:g}" height="38" '
                           'rx="19" fill="#f6fbff" stroke="#cfdeeb" stroke-width="1.7"/>')
        self.labels.append(self.text(x,y+3,text,size,True))

    def note(self, x, y, text, size=20):
        self.labels.append(self.text(x,y,text,size,False,'#536c7f'))

    def edge(self, points, dashed=False):
        # Rounded orthogonal polylines, drawn below nodes and edge labels.
        assert len(points) >= 2
        d = f'M {points[0][0]:g} {points[0][1]:g}'
        for i in range(1,len(points)-1):
            a,b,c = points[i-1], points[i], points[i+1]
            da,dc = math.dist(a,b),math.dist(b,c)
            if not da or not dc:
                continue
            r=min(13,da/2,dc/2)
            before=(b[0]+(a[0]-b[0])*r/da,b[1]+(a[1]-b[1])*r/da)
            after=(b[0]+(c[0]-b[0])*r/dc,b[1]+(c[1]-b[1])*r/dc)
            d+=f' L {before[0]:g} {before[1]:g} Q {b[0]:g} {b[1]:g} {after[0]:g} {after[1]:g}'
        d+=f' L {points[-1][0]:g} {points[-1][1]:g}'
        self.edges.append(f'<path d="{d}" fill="none" stroke="{LINE}" stroke-width="2.1" '
                          'stroke-linejoin="round" marker-end="url(#arrow)"'
                          + (' stroke-dasharray="9 7"' if dashed else '') + '/>')

    def svg(self):
        return '\n'.join([
            '<?xml version="1.0" encoding="UTF-8"?>',
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
            f'viewBox="0 0 {self.w} {self.h}" role="img" aria-labelledby="title desc">',
            f'<title id="title">{escape(self.title)}</title>',
            f'<desc id="desc">{escape(self.subtitle)}。详细职责与源码见 docs/architecture.md。</desc>',
            '<defs><marker id="arrow" markerWidth="10" markerHeight="10" refX="8" refY="5" '
            'orient="auto" markerUnits="userSpaceOnUse"><path d="M 1 1 L 8 5 L 1 9" '
            f'fill="none" stroke="{LINE}" stroke-width="1.7"/></marker></defs>',
            f'<rect width="{self.w}" height="{self.h}" fill="#fcfdff"/>',
            f'<g font-family="{FONT}">',
            self.text(self.w/2,55,self.title,34,True),
            self.text(self.w/2,96,self.subtitle,21,False,'#536c7f'),
            *self.groups,*self.edges,*self.nodes,*self.labels,
            self.text(self.w/2,self.h-27,'实线：数据 / 控制流    ·    虚线：启动选择 / 可选关系    ·    依据当前 launch 与配置',18,False,'#64798b'),
            '</g></svg>',''])


def badge(label, value, color):
    """A self-contained README badge. Text widths also fix the fallback font layout."""
    left = math.ceil(units(label) * 12) + 16
    right = math.ceil(units(value) * 12) + 16
    width = left + right
    title = escape(f'{label}: {value}')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="22" '
            f'viewBox="0 0 {width} 22" role="img"><title>{title}</title>'
            f'<clipPath id="clip"><rect width="{width}" height="22" rx="4"/></clipPath>'
            f'<g clip-path="url(#clip)"><rect width="{left}" height="22" fill="#334453"/>'
            f'<rect x="{left}" width="{right}" height="22" fill="{color}"/></g>'
            '<g font-family="DejaVu Sans,Verdana,sans-serif" font-size="11" fill="white" '
            'text-anchor="middle">'
            f'<text x="{left/2:g}" y="15">{escape(label)}</text>'
            f'<text x="{left+right/2:g}" y="15">{escape(value)}</text></g></svg>\n')


def warehouse_cover():
    """Draw the actual static collision footprints and six map-frame mission goals."""
    world = ROOT/'src/navcraft_simulation/worlds/indoor_room.sdf.xacro'
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 280" role="img" '
        'aria-labelledby="title desc">',
        '<title id="title">NavCraft · Warehouse navigation</title>',
        '<desc id="desc">Six numbered mission stops on the simulated warehouse floor plan, '
        'with positive map Y pointing up.</desc>',
        '<defs><pattern id="grid" width="23" height="23" patternUnits="userSpaceOnUse">'
        '<path d="M23 0H0V23" fill="none" stroke="#32485b" stroke-width="0.8"/>'
        '</pattern></defs>',
        '<rect width="1000" height="280" rx="15" fill="#101e2d"/>',
        '<g font-family="DejaVu Sans,Arial,sans-serif">',
        '<text x="38" y="100" font-size="64" font-weight="700" fill="#f6f8fa">NavCraft</text>',
        '<text x="42" y="139" font-size="23" fill="#b9ccda">Warehouse navigation simulator</text>',
        '<path d="M42 168H430" stroke="#3b5264"/>',
        '<text x="42" y="202" font-size="17" fill="#76dfc4">Six-stop warehouse mission</text>',
        '<text x="42" y="230" font-size="15" fill="#c4d3df">Known-map navigation · SLAM · Path tracking</text>',
        '<g transform="translate(621 140)">',
        '<rect x="-128" y="-128" width="256" height="256" rx="6" fill="url(#grid)" '
        'stroke="#3b5264"/>',
        '<g transform="scale(23 -23)" fill="#718999">',
    ]
    for x, y, width, height, yaw, _ in obstacles(world):
        svg.append(f'<rect x="{-width/2:g}" y="{-height/2:g}" width="{width:g}" '
                   f'height="{height:g}" transform="translate({x:g} {y:g}) '
                   f'rotate({math.degrees(yaw):g})"/>')
    svg.append('</g>')
    for index, (_, (x, y)) in enumerate(STATIONS, 1):
        svg.append(f'<circle cx="{23*x:g}" cy="{-23*y:g}" r="9" fill="#85e5c9" stroke="#101e2d"/>')
        svg.append(f'<text x="{23*x:g}" y="{-23*y+4:g}" text-anchor="middle" '
                   f'font-size="11" font-weight="700" fill="#132c31">{index}</text>')
    svg += ['</g>', '<g transform="translate(777 48)">',
            '<text font-size="12" letter-spacing="1.5" fill="#90a8ba">MISSION STOPS</text>']
    for index, (name, _) in enumerate(STATIONS, 1):
        y = 30 * index
        svg.append(f'<text y="{y}" font-size="15" fill="#85e5c9">{index:02d}</text>')
        svg.append(f'<text x="32" y="{y}" font-size="16" fill="#edf2f6">'
                   f'{escape(name.replace("_", " ").title())}</text>')
    svg += ['</g></g></svg>', '']
    return '\n'.join(svg)


def overview():
    # Keep the README overview readable; detailed sensor/TF graphs live in docs.
    d=Diagram(1440,1170,'NavCraft · 总体架构',
              '三种模式互斥运行，共用 Gazebo 仓库与差速机器人')
    d.box(440,130,560,110,'统一入口 · demo.sh up',
          ['navcraft_bringup · mode 三选一'])
    for x,title in [(40,'navigation · 已有地图导航'),
                    (510,'slam · 激光建图'),
                    (980,'tracking · 独立路径跟踪')]:
        d.group(x,310,420,535,title)
    for x in [250,720,1190]:
        d.edge([(720,240),(720,275),(x,275),(x,310)],True)
    d.box(60,370,380,135,'航点任务 · AMCL · Nav2',
          ['地图 + 激光 + 融合里程计','激光 / 深度避障，执行航点任务'])
    d.box(530,370,380,135,'SLAM Toolbox / Cartographer',
          ['/scan + odom / TF → /map','只建图，不产生运动速度'])
    d.box(1000,370,380,135,'Pure Pursuit / Stanley',
          ['/reference_path + /odom','跟随预设路径，无自主避障'])
    d.box(530,555,380,100,'键盘遥控', ['/teleop_cmd_vel'])
    d.box(60,700,380,125,'导航速度保护',
          ['navcraft_safety','任务状态 · 超时 · 急停'])
    for x in [530,1000]:
        d.box(x,700,380,125,'仲裁与速度保护',
              ['twist_mux → navcraft_teleop','输入超时 · 急停'])
    d.edge([(250,505),(250,700)])
    d.pill(250,590,'/nav2_cmd_vel + 状态',18)
    d.edge([(720,655),(720,700)])
    d.edge([(1190,505),(1190,700)])
    d.pill(1190,590,'/tracking_cmd_vel',19)
    # Separate arrows converge only below the mutually exclusive modes.
    for x in [250,720,1190]:
        d.edge([(x,825),(x,885),(720,885),(720,955)])
    d.pill(720,887,'/cmd_vel · 唯一最终发布者',21)
    d.box(240,955,960,125,'Gazebo 仓库 · 差速机器人',
          ['/cmd_vel → ros_gz_bridge → DiffDrive',
           '导航反馈：轮速 + IMU → /odom · 激光 + 深度 → 代价地图'])
    d.note(720,1118,'建图与独立跟踪使用轮式里程计。Serial / UDP / mock / ros2_control 另行启动。',19)
    return d.svg()


def warehouse():
    d=Diagram(1800,2680,'NavCraft · 仓库导航、绕障与独立验收',
              '激光、深度与融合里程计 → 定位与代价地图 → Nav2 规划控制 → 速度保护 → Gazebo；真值只用于测量')
    d.box(80,155,320,120,'放置实体障碍',['obstacle block / clear','或 test_warehouse.py 自动注入'])
    d.box(1250,155,460,120,'warehouse 命令',['mission.py → /navigate_sequence'])
    d.box(80,380,320,145,'激光与前向深度',['/scan · /camera/scan','实时检测障碍表面'])
    d.box(480,380,350,145,'轮速 + IMU → EKF',['/odom：控制用运动估计','odom → base_footprint：定位输入'])
    d.box(900,380,300,145,'静态仓库地图',['map_server → /map','由场景碰撞几何生成'])
    d.box(1250,380,460,145,'六站路线 warehouse_route.py',
          ['receiving → storage B → storage A','packing → dock 接近点 → home'])
    d.edge([(240,275),(240,380)])
    d.pill(240,330,'障碍观测')
    d.edge([(1480,275),(1480,380)])
    d.box(50,690,400,190,'全局 / 局部代价地图',
          ['静态层 + 障碍层 + 膨胀层','激光与深度扫描共同更新','全局 map 帧；局部 odom 滚动窗口'])
    d.box(520,690,400,190,'AMCL 定位',
          ['仅 /scan 匹配已有地图','经 TF 获取里程计运动','输出 map → odom'])
    d.box(1150,690,560,200,'航点执行器 navcraft_mission',
          ['顺序派发 · 有限重试 · 超时处理','暂停 / 恢复 / 取消 · 迟到回调收尾','下层结果未知 → FAULT，拒绝新任务'])
    d.edge([(200,525),(200,690)])
    d.edge([(300,525),(300,590),(560,590),(560,690)])
    d.edge([(655,525),(655,690)])
    d.edge([(1020,525),(1020,610),(840,610),(840,690)])
    d.edge([(1100,525),(1100,570),(400,570),(400,690)])
    d.edge([(1480,525),(1480,690)])
    d.group(380,1100,1050,545,'Nav2 导航：BT 调度、全局规划、路径跟随与恢复')
    d.box(430,1185,570,155,'全局规划 planner_server',
          ['NavFn：Dijkstra 搜索（use_astar=false）','全局代价地图 → 可行路径 /plan'])
    d.box(1080,1185,300,155,'BT 导航调度',['bt_navigator','NavigateToPose Action'])
    d.box(430,1430,570,160,'路径跟随 controller_server',
          ['Regulated Pure Pursuit · 20 Hz','/plan + /odom + TF + 局部代价地图','速度调节与碰撞检测，目标速度 0.35 m/s'])
    d.box(1080,1430,300,160,'恢复行为',['behavior_server','旋转 / 后退 / 等待'])
    d.edge([(250,880),(250,1260),(430,1260)])
    d.edge([(310,880),(310,1510),(430,1510)])
    d.edge([(720,880),(720,1100)])
    d.pill(720,985,'地图中的机器人位姿：map → odom + odom TF',20)
    d.edge([(1350,890),(1350,1070),(1230,1070),(1230,1185)])
    d.pill(1350,985,'/navigate_to_pose',20)
    d.edge([(1080,1260),(1000,1260)])
    d.edge([(1080,1310),(1040,1310),(1040,1385),(940,1385),(940,1430)])
    d.pill(1030,1385,'FollowPath',18)
    d.edge([(1230,1340),(1230,1430)])
    d.edge([(715,1340),(715,1430)])
    d.pill(715,1387,'/plan',21)
    # /odom is a separate message path into Nav2; AMCL's input is TF.
    d.edge([(480,450),(465,450),(465,1025),(350,1025),(350,1558),(430,1558)])
    d.pill(460,1025,'/odom',20)
    d.box(650,1745,650,105,'Nav2 速度 /nav2_cmd_vel',['路径跟随或恢复行为产生 Twist'])
    d.edge([(715,1590),(715,1680),(850,1680),(850,1745)])
    d.edge([(1230,1590),(1230,1680),(1120,1680),(1120,1745)])
    d.box(1470,1745,260,105,'导航急停',['/emergency_stop'])
    d.box(620,1970,850,195,'navcraft_safety · 导航速度门',
          ['仅 RUNNING 放行；状态心跳与速度各自 < 300 ms','锁存急停 · 拒绝非有限值 · 限幅 ±0.5 m/s、±1.2 rad/s','暂停 / 取消 / FAULT / 超时 → 零速；解除急停需要新速度'])
    d.edge([(975,1850),(975,1970)])
    d.edge([(1600,1850),(1600,1915),(1410,1915),(1410,1970)])
    d.edge([(1610,890),(1610,1705),(1760,1705),(1760,1885),(1320,1885),(1320,1970)])
    d.pill(1610,1620,'/execution_state',20)
    d.box(620,2290,520,130,'Gazebo 差速机器人',['/cmd_vel → bridge → DiffDrive','运动产生轮速、IMU、激光与深度观测'])
    d.edge([(1045,2165),(1045,2230),(880,2230),(880,2290)])
    d.pill(1045,2230,'唯一最终输出 /cmd_vel',20)
    d.box(1280,2290,450,130,'Gazebo 世界位姿',['PosePublisher · 20 Hz','/model/mobile_robot/pose'])
    d.edge([(1140,2355),(1280,2355)])
    d.box(620,2500,1110,125,'独立验收 test_warehouse.py',
          ['测试专用桥接 /warehouse_ground_truth；绝不注入导航 /tf','六站 map / world 误差 < 0.35 m；障碍中心距离 > 0.40 m；完结零速'])
    d.edge([(1505,2420),(1505,2500)])
    d.box(65,2310,400,280,'观测与验收边界',
          ['/plan：全局路径','/lookahead_collision_arc：碰撞弧','odom 误差单独记录','世界位姿不反馈给导航','warehouse 不自动放置障碍'],small=True)
    d.edge([(620,2355),(550,2355),(550,2240),(25,2240),(25,450),(80,450)])
    d.pill(260,2240,'运动后重新感知',20)
    return d.svg()


def sensor_dataflow():
    d=Diagram(1800,2180,'NavCraft · 逐条传感器与运动反馈数据链',
              '每一行独立表示来源、桥接、处理和实际消费者；频率为配置值')
    for x,label in [(215,'Gazebo / 数据来源'),(635,'ROS 桥接'),(1060,'处理或输出'),(1560,'实际用途 / 边界')]:
        d.note(x,165,label,24)
    rows=[
        ('激光雷达 gpu_lidar',['10 Hz · 360 样本','约 ±π · 0.08–12 m'],
         'ros_gz_bridge',['/sim/scan','sensor_msgs/LaserScan'],
         'laser_filter_node',['规范帧与无效距离 / 角度','/scan · lidar_link'],
         '定位、代价地图与建图',['AMCL / Nav2 / 两套 SLAM','也供 RViz 与仿真检查']),
        ('IMU',['100 Hz','姿态、角速度、线加速度'],
         'ros_gz_bridge',['/sim/imu','sensor_msgs/Imu'],
         'imu_filter_node',['规范帧，补全未知角速度方差','/imu/data · imu_link'],
         '默认导航 EKF',['偏航角速度 + 轮式速度','/odom 与 odom TF']),
        ('RGB-D 相机',['15 Hz · 640×480','水平 FOV 1.2 rad'],
         'ros_gz_bridge',['深度图 + CameraInfo','RGB 预览 /sim/camera/image'],
         'depthimage_to_laserscan',['中心扫描带 → /camera/scan','平面扫描帧 camera_link'],
         'Nav2 障碍层',['前向深度与激光共同更新','全局 / 局部代价地图']),
        ('轮式里程计 DiffDrive',['轮半径 0.075 m','轮距 0.43 m'],
         'ros_gz_bridge',['/model/mobile_robot/odometry','nav_msgs/Odometry'],
         'gazebo_odom_tf_node',['导航 → /wheel/odom → EKF','建图 / 跟踪 → /odom 与 TF'],
         '运动估计与坐标反馈',['Nav2 / Cartographer / 独立跟踪','AMCL、SLAM Toolbox 经 TF 使用']),
        ('仿真时钟',['Gazebo simulation time','非墙钟 / 非到站真值'],
         'ros_gz_bridge',['/clock','rosgraph_msgs/Clock'],
         'ROS 仿真时间',['use_sim_time=true 的节点','消息时间戳与仿真同步'],
         '时间基准',['导航门的新鲜度检查','另用 steady_clock 单调时钟']),
        ('世界位姿 PosePublisher',['20 Hz · 模型 mobile_robot','世界 indoor_room'],
         '仅验收时额外桥接',['/model/mobile_robot/pose','gz.msgs.Pose_V → TFMessage'],
         '/warehouse_ground_truth',['test_warehouse.py 启动桥接','与导航 /tf 完全分离'],
         '独立世界坐标验收',['六站到站 / 绕障中心距离','同时记录 map 与 odom 差异']),
    ]
    for i,row in enumerate(rows):
        y=210+i*275
        d.group(30,y-15,1740,220,'')
        for j,x in enumerate([50,460,880,1370]):
            d.box(x,y+15,[330,350,380,380][j],175,row[j*2],row[j*2+1],small=True)
        for a,b in [(380,460),(810,880),(1260,1370)]:d.edge([(a,y+102),(b,y+102)])
    d.box(200,1930,1400,155,'可选硬件入口 · hardware_ekf.launch.py',
          ['/wheel/odom + /imu/data → robot_localization/ekf_node → /odom 与 odom TF',
           '启用时关闭原始底盘 odom / TF 发布；不能与 Gazebo odom 适配叠加'],optional=True)
    return d.svg()


def tf_ownership():
    d=Diagram(1800,1930,'NavCraft · TF 树与发布责任',
              'TF 是坐标关系；每条动态变换只保留一个发布路径，世界真值不进入这棵树')
    d.box(650,165,500,135,'map',['导航：AMCL','建图：SLAM Toolbox / Cartographer'])
    d.box(650,440,500,110,'odom',['连续里程计参考系'])
    d.edge([(900,300),(900,440)])
    d.pill(900,367,'map → odom：定位 / 建图二选一',20)
    d.box(650,690,500,110,'base_footprint',['底盘在地面上的投影坐标系'])
    d.edge([(900,550),(900,690)])
    d.pill(900,617,'导航：navigation_ekf · 建图 / 跟踪：gazebo_odom_tf_node',20)
    d.box(650,940,500,110,'base_link',['机器人实体参考系'])
    d.edge([(900,800),(900,940)])
    d.pill(900,867,'robot_state_publisher · 固定 z = 0.075 m',20)
    children=[
        (45,310,'lidar_link',['xyz=(0.18, 0, 0.27) m','固定安装 · /tf_static']),
        (395,310,'imu_link',['xyz=(0, 0, 0.22) m','固定安装 · /tf_static']),
        (745,310,'camera_link',['xyz=(0.28, 0, 0.20) m','子帧 camera_optical_frame']),
        (1095,310,'caster_wheel',['xyz=(-0.20, 0, -0.035) m','固定安装 · /tf_static']),
        (1445,310,'left / right_wheel',['可动轮关节 · /tf','当前 joint_states 供显示']),
    ]
    for x,w,title,lines in children:
        d.box(x,1260,w,155,title,lines,small=True)
        d.edge([(900,1050),(900,1160),(x+w/2,1160),(x+w/2,1260)])
    d.pill(900,1160,'robot_state_publisher + URDF',21)
    d.note(900,1485,'安装 xyz 相对 base_link；joint_state_publisher 未接入 Gazebo 真实轮关节反馈',22)
    d.box(60,1595,800,220,'可选底盘：替换 odom 发布责任',
          ['直接驱动：底盘节点提供 odom 与 TF','EKF：关闭驱动发布，由 EKF 独占 odom TF',
           'ros2_control：差速控制器负责 odom TF','三者均为独立入口，不与默认 Gazebo 同开'],optional=True)
    d.box(940,1595,800,220,'与导航 TF 分开的关系',
          ['独立跟踪只用 odom，无需 map → odom','PosePublisher → /warehouse_ground_truth 仅验收',
           '绝不把 Gazebo 世界位置灌入 /tf','轮式 odom 偏离需单独评价，不能以真值替换'],optional=True)
    return d.svg()


def outputs():
    version = ET.parse(ROOT/'src/navcraft_simulation/package.xml').findtext('version')
    return {'badge-ros.svg': badge('ROS 2', 'Jazzy', '#315b7a'),
            'badge-gazebo.svg': badge('Gazebo', 'Harmonic', '#99570c'),
            'badge-ubuntu.svg': badge('Ubuntu', '24.04', '#ad4429'),
            'badge-version.svg': badge('version', 'v' + version, '#26695e'),
            'badge-license.svg': badge('license', 'PolyForm Noncommercial 1.0.0', '#64518d'),
            'warehouse-cover.svg':warehouse_cover(),
            'architecture-overview.svg':overview(),
            'warehouse-navigation.svg':warehouse(),
            'sensor-dataflow.svg':sensor_dataflow(),
            'tf-ownership.svg':tf_ownership()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true',help='fail on missing/stale generated SVGs')
    args=parser.parse_args()
    folder=ROOT/'docs/images'
    if not args.check: folder.mkdir(parents=True,exist_ok=True)
    stale=[]
    for name,content in outputs().items():
        path=folder/name
        if args.check:
            if not path.is_file() or path.read_text(encoding='utf-8')!=content: stale.append(name)
        else: path.write_text(content,encoding='utf-8')
    if stale:
        print('Missing/stale SVGs: '+', '.join(stale))
        return 1
    print('PASS: documentation SVGs '+('match source.' if args.check else 'generated.'))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
