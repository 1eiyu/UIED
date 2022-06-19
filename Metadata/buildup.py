#!/usr/bin/env python
# -*- coding:utf-8 -*-
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))
from Meta import UserInterface
from run_single import run
import detect_text_east.lib_east.eval as eval
from cnn.CNN import CNN
from collections import deque
from os.path import join as pjoin

import os

def naming(prefix, h, w):
    return str(prefix) + '_' + str(h) + '_' + str(w)


def bfs(ui_dict, cur_ui):
    d = deque([cur_ui.name])
    visited = set(cur_ui.name)
    while d:
        infos = d.popleft()
        ui_name, *path = infos
        ui = ui_dict[ui_name]
        if ui.compo_cursor < ui.compo_nums:
            path.append(ui.compo_cursor)
            return path
        else:
            for compo_id, ui_name in ui.sub_ui_images.items():
                if ui.name not in visited:
                    d.append([ui.name] + path + [compo_id])
    return []


def saving(root_path):
    root = root_path
    for name, ui in UI_dict.items():
        os.makedirs(pjoin(root, name), exist_ok=True)
        for compo_id, ui_name in ui.sub_ui_images.items():
            os.makedirs(pjoin(root, name, compo_id), exist_ok=True)
            os.makedirs(pjoin(root, name, compo_id, ui_name), exist_ok=True)


if __name__ == '__main__':
    save_path = '/Users/yulei/Downloads/UIED-2.3/data/volvo'
    print("Save Path: " + save_path + '\n')

    ui_id = 0
    UI_dict = {}
    models = eval.load()
    classifier = {'Elements': CNN('Elements')}
    searching_end = False
    prev_meta = None

    while True:
        idx = input()
        dist_h = dist_w = 0
        img_path = '/Users/yulei/Downloads/UIED-2.3/data/input/' + idx + '.png'
        if len(idx) > 5:
            idx = idx[5:]
        org_resized, ui_compos = run(img_path, save_path, models, classifier, int(idx))
        meta = UserInterface(naming(ui_id, dist_h, dist_w), org_resized, ui_compos)
        #
        # if meta.is_same(prev_meta):
        #     print("Last button is a noise!)")
        #     prev_meta.update_next_button()
        #     if prev_meta.end_search():
        #         path = bfs(UI_dict, prev_meta)
        #         if path:
        #             print(path)
        #         else:
        #             print('All buttons are found.')
        #             break
        #     else:
        #         button = prev_meta.get_next_button()
        #         h, w = button.bbox.mid_point()
        #         print('An old UI Image, robot should press: (%d, %d)' % (h, w))
        #     print('robot is opearting......')
        #     continue
        #
        # for ui in UI_dict.values():
        #     if ui.name == prev_meta.name:
        #         continue
        #     if meta.is_same(ui):
        #         if prev_meta:
        #             prev_meta.save_connection(prev_meta.compo_cursor, ui)
        #         if ui.end_search():
        #             path = bfs(UI_dict, ui)
        #             if path:
        #                 print((path))
        #                 print('robot is opearting......')
        #             else:
        #                 print('All buttons are found.')
        #                 searching_end = True
        #                 break
        #         else:
        #             button = ui.get_next_button()
        #             h, w = button.bbox.mid_point()
        #             prev_meta = ui
        #             print('An old UI Image, robot should press: (%d, %d)' % (h, w))
        #             print('robot is opearting......')
        # if searching_end:
        #     break
        #
        # if prev_meta:
        #     prev_meta.save_connection(prev_meta.compo_cursor, meta)
        # UI_dict[meta.name] = meta
        # if meta.has_slide_bar:
        #     bars = meta.find_slide_bar()
        #     # here supposed: only one bar at most for each UI Image
        #     bar, bar_id = bars[0]
        #     img_height, img_width = org_resized.shape[:2]
        #     bar_height, bar_width = bar.height, bar.width
        #     col_min, row_min, col_max, row_max = bar.bbox.put_bbox()
        #     finger_start_point = bar.bbox.mid_point()
        #     # (height, width)
        #     finger_end_point = None
        #     while True:
        #         print('start at:', finger_start_point)
        #         if bar_height > bar_width:
        #             step = int(bar_height * 0.9)
        #             dist_h += step
        #             finger_end_point = (finger_start_point[0] + step, finger_start_point[1])
        #         else:
        #             step = int(bar_width * 0.9)
        #             dist_w += step
        #             finger_end_point = (finger_start_point[0], finger_start_point[1] + step)
        #         print('should end at:', finger_end_point)
        #         # add some function here, which lets the finger move and wait the feedback
        #         print('Robot is operating......')
        #         img_path = input()
        #         sub_org, sub_compos = run(img_path, save_path, models, classifier)
        #         sub_meta = UserInterface(naming(ui_id, dist_h, dist_w), sub_org, sub_compos)
        #         new_bar, new_bar_id = meta.find_slide_bar()[0]
        #         if new_bar.bbox.mid_point() < finger_end_point:
        #             break
        #         UI_dict[sub_meta.name] = sub_meta
        #         meta.save_connection(- bar_id, sub_meta.name)
        #         finger_start_point = new_bar.bbox.mid_point
        #         meta = sub_meta
        #     print('Silder finished moving')
        # prev_meta = meta
        # first_button = meta.get_first_button()
        # h, w = first_button.bbox.mid_point()
        # print('A new UI Image, robot should press: (%d, %d)' % (h, w))
        # print('Robot is operating......')

