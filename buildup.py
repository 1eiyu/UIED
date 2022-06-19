#!/usr/bin/env python
# -*- coding:utf-8 -*-
from Meta import UserInterface
from run_single import run
from detect_text_east.lib_east import eval
from cnn.CNN import CNN
from collections import deque
from os.path import join as pjoin
import os, cv2

import actionlib
import rospy
import ctl_interface_msgs.msg


def clickClient(x_rel,y_rel):

    client = actionlib.SimpleActionClient('press_screen_coordinate',ctl_interface_msgs.msg.PressScreenCoordinateAction)

    client.wait_for_server()

    goal = ctl_interface_msgs.msg.PressScreenCoordinateGoal(x_coordinate_relative=x_rel,y_coordinate_relative=y_rel)

    client.send_goal(goal)

    client.wait_for_result()

    print(client.get_result)


def naming(prefix, h, w):
    return str(prefix) + '_' + str(h) + '_' + str(w)


def bfs(ui_dict, cur_ui):
    d = deque([[cur_ui.name]])
    visited = set()
    while d:
        infos = d.popleft()
        ui_name, *path = infos
        print(ui_name, path)
        visited.add(ui_name)
        current_ui = ui_dict[ui_name]
        if current_ui.compo_cursor < current_ui.compo_nums:
            compo = current_ui.get_next_button()
            coordinate = compo.bbox.mid_point()
            path.append(coordinate)
            return path
        else:
            for new_ui_name in current_ui.sub_ui_images.values():
                if new_ui_name not in visited:
                    next_ui = ui_dict[new_ui_name]
                    compo = next_ui.get_next_button()
                    coordinate = compo.bbox.mid_point()
                    d.append([new_ui_name] + path + [coordinate])
    return []


def saving(root_path):
    root = root_path
    for name, ui in UI_dict.items():
        os.makedirs(pjoin(root, name), exist_ok=True)
        for compo_id, ui_name in ui.sub_ui_images.items():
            os.makedirs(pjoin(root, name, compo_id), exist_ok=True)
            os.makedirs(pjoin(root, name, compo_id, ui_name), exist_ok=True)


if __name__ == '__main__':
    cur_path = os.path.abspath('.')
    save_path = os.path.join(cur_path, 'data')
    
    print("Save Path: " + save_path + '\n')

    ui_id = 0
    UI_dict = {}
    models = eval.load()
    classifier = {'Elements': CNN('Elements')}
    searching_end = False
    prev_meta = None

    while True:
        dist_h = dist_w = 0

        """
        change interface here.
        """
        img_name = '0.png'
        
        img_path = os.path.join(cur_path, 'data', 'input', img_name)
        org = cv2.imread(img_path)
        H, W = org.shape[:2]
        org_resized, ui_compos = run(img_path, save_path, models, classifier)
        meta = UserInterface(naming(ui_id, dist_h, dist_w), org_resized, ui_compos)

        found_same = False
        if meta.is_same(prev_meta):
            found_same = True
            print("Last button is a noise!")
            if prev_meta.end_search():
                path = bfs(UI_dict, prev_meta)
                if path:
                    print(path)
                    print('robot is opearting......')
                    for h, w in path:
                        try:
                            rospy.init_node('interface_tester_py')
                            clickClient(h / H, w / W)
                        except rospy.ROSInterruptException:
                            print("program interrupted")
                else:
                    print('All buttons are found.')
                    break
            else:
                button = prev_meta.get_next_button()
                h, w = button.bbox.mid_point()
                print('An old UI Image, robot should press: (%d, %d)' % (h, w))
                try:
                    rospy.init_node('interface_tester_py')
                    clickClient(h / H, w / W)
                except rospy.ROSInterruptException:
                    print("program interrupted")
        if found_same:
            continue

        for ui in UI_dict.values():
            if ui.name == prev_meta.name:
                continue
            if meta.is_same(ui):
                if prev_meta:
                    prev_meta.save_connection(prev_meta.compo_cursor - 1, ui.name)
                if ui.end_search():
                    path = bfs(UI_dict, ui)
                    if path:
                        print((path))
                        print('robot is opearting......')
                        for h, w in path:
                            try:
                                rospy.init_node('interface_tester_py')
                                clickClient(h / H, w / W)
                            except rospy.ROSInterruptException:
                                print("program interrupted")
                    else:
                        print('All buttons are found.')
                        searching_end = True
                        break
                else:
                    button = ui.get_next_button()
                    h, w = button.bbox.mid_point()
                    prev_meta = ui
                    print('An old UI Image, robot should press: (%d, %d)' % (h, w))
                    print('robot is opearting......')
                    try:
                        rospy.init_node('interface_tester_py')
                        clickClient(h / H, w / W)
                    except rospy.ROSInterruptException:
                        print("program interrupted")
                found_same = True
            if found_same:
                break
        if found_same:
            continue
        if searching_end:
            break

        if prev_meta:
            prev_meta.save_connection(prev_meta.compo_cursor - 1, meta.name)
        UI_dict[meta.name] = meta
        # if meta.has_slide_bar:
        #     bars = meta.find_slide_bar()
        #     # here supposed: only one bar at most for each UI Image
        #     bar, bar_id = bars[0]
        #     img_height, img_width = org_resized.shape[:2]
        #     bar_height, bar_width = bar.height, bar.width
        #     col_min, row_min, col_max, row_max = bar.bbox.put_bbox()
            # finger_start_point = bar.bbox.mid_point()
            # # (height, width)
            # finger_end_point = None
            # while True:
            #     print('start at:', finger_start_point)
            #     if bar_height > bar_width:
            #         step = int(bar_height * 0.9)
            #         dist_h += step
            #         finger_end_point = (finger_start_point[0] + step, finger_start_point[1])
            #     else:
            #         step = int(bar_width * 0.9)
            #         dist_w += step
            #         finger_end_point = (finger_start_point[0], finger_start_point[1] + step)
            #     print('should end at:', finger_end_point)
            #     # add some function here, which lets the finger move and wait the feedback
            #     print('Robot is operating......')
            #     img_path = input()
            #     sub_org, sub_compos = run(img_path, save_path, models, classifier)
            #     sub_meta = UserInterface(naming(ui_id, dist_h, dist_w), sub_org, sub_compos)
            #     new_bar, new_bar_id = meta.find_slide_bar()[0]
            #     if new_bar.bbox.mid_point() < finger_end_point:
            #         break
            #     UI_dict[sub_meta.name] = sub_meta
            #     meta.save_connection(- bar_id, sub_meta.name)
            #     finger_start_point = new_bar.bbox.mid_point
            #     meta = sub_meta
            # print('Silder finished moving')
        prev_meta = meta
        first_button = meta.get_first_button()
        h, w = first_button.bbox.mid_point()
        print('A new UI Image, robot should press: (%d, %d)' % (h, w))
        print('Robot is operating......')
        try:
            rospy.init_node('interface_tester_py')
            clickClient(h / H, w / W)
        except rospy.ROSInterruptException:
            print("program interrupted")
        ui_id += 1

