#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gear_then_external.py (final)
- 시작: P
- 순서: P 브레이크 유지 → D 변속(option=2) → External(=ctrl_mode=3) 전환(option=1) → MPC 제어 시작
- MORAI EventCmd: option {1: ctrl_mode, 2: gear}
- MPC가 바로 제어를 시작하도록 수정됨 (0.5초 전진 제거)
"""

import time
import rospy
from morai_msgs.msg import CtrlCmd
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvRequest

def _fill_event_req(req, option=None, gear=None, ctrl_mode=None, set_pause=None):
    """버전별로 request 필드명이 다른 경우를 방지해 유연하게 채움."""
    target = req
    if hasattr(req, 'request'):
        target = req.request

    if option is not None:
        if hasattr(target, 'option'):     target.option = int(option)
        elif hasattr(target, 'cmd_type'): target.cmd_type = int(option)

    if gear is not None:
        if hasattr(target, 'gear'):   target.gear = int(gear)
        elif hasattr(target, 'value'): target.value = int(gear)

    if ctrl_mode is not None and hasattr(target, 'ctrl_mode'):
        target.ctrl_mode = int(ctrl_mode)

    if set_pause is not None and hasattr(target, 'set_pause'):
        target.set_pause = bool(set_pause)

def call_event_cmd(srv_name, option, *, gear=None, ctrl_mode=None, set_pause=None, timeout=3.0):
    rospy.wait_for_service(srv_name, timeout=timeout)
    cli = rospy.ServiceProxy(srv_name, MoraiEventCmdSrv)
    req = MoraiEventCmdSrvRequest()
    _fill_event_req(req, option=option, gear=gear, ctrl_mode=ctrl_mode, set_pause=set_pause)
    res = cli(req)
    return res

def publish_ctrl(pub, steer, accel, brake, duration_sec, rate_hz):
    r = rospy.Rate(rate_hz)
    t0 = time.time()
    msg = CtrlCmd()
    # 필요시 cmd_type/longlCmdType 등을 추가로 세팅 가능
    while not rospy.is_shutdown() and (time.time() - t0) < duration_sec:
        msg.steering = float(steer)
        msg.accel    = float(accel)
        msg.brake    = float(brake)
        pub.publish(msg)
        r.sleep()

def main():
    rospy.init_node('gear_then_external_node')

    # 파라미터
    ctrl_cmd_topic = rospy.get_param('~ctrl_cmd_topic', '/ctrl_cmd')
    event_srv      = rospy.get_param('~event_srv', '/Service_MoraiEventCmd')
    gear_p         = int(rospy.get_param('~gear_p', 1))
    gear_d         = int(rospy.get_param('~gear_d', 4))
    ctrl_external  = int(rospy.get_param('~ctrl_mode_external', 3))  # 보통 3(automode)이 External용
    hold_p_sec     = float(rospy.get_param('~hold_p_sec', 0.8))
    hold_d_sec     = float(rospy.get_param('~hold_d_sec', 0.5))
    pub_rate       = float(rospy.get_param('~pub_rate', 20.0))

    pub = rospy.Publisher(ctrl_cmd_topic, CtrlCmd, queue_size=10)
    rospy.sleep(0.3)

    rospy.loginfo("Step 1: P 상태에서 브레이크 유지")
    publish_ctrl(pub, steer=0.0, accel=0.0, brake=1.0, duration_sec=hold_p_sec, rate_hz=pub_rate)

    rospy.loginfo("Step 2: D 기어로 변속 (EventCmd option=2)")
    try:
        res = call_event_cmd(event_srv, option=2, gear=gear_d)
        rospy.loginfo(" -> gear 현재상태: %s", getattr(res, 'gear', getattr(getattr(res, 'response', None), 'gear', '?')))
    except Exception as e:
        rospy.logwarn("D 변속 실패: %s", e)

    rospy.loginfo("Step 3: External(automode)로 제어모드 전환 (EventCmd option=1, ctrl_mode=%d)", ctrl_external)
    try:
        res = call_event_cmd(event_srv, option=1, ctrl_mode=ctrl_external)
        rospy.loginfo(" -> ctrl_mode 현재상태: %s", getattr(res, 'ctrl_mode', getattr(getattr(res, 'response', None), 'ctrl_mode', '?')))
    except Exception as e:
        rospy.logwarn("ctrl_mode 전환 실패: %s", e)

    # Step 4 제거: MPC가 바로 제어 시작하도록 함
    # 기어 D + External 모드 전환 완료 후 MPC에게 제어권 넘김
    rospy.loginfo("Step 4: MPC 제어 시작 대기")
    rospy.loginfo("Done - MPC should now take control")

if __name__ == '__main__':
    main()
