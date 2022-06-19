#!/usr/bin/env python
# -*- coding:utf-8 -*-

class UserInterface:
    def __init__(self, ui_name, org_img, components):
        self.name = ui_name
        self.img = org_img
        for compo in components:
            w, h = compo.width, compo.height
            if h / w > 20:
                compo.category = 'SlideBar'
        self.compos = sorted(components, key=lambda compo:(compo.bbox.col_min, compo.bbox.row_min))
        self.sub_ui_images = {}
        self.compo_nums = len(self.compos)
        self.has_slide_bar = any(compo.category == 'SlideBar' for compo in self.compos)
        self.compo_cursor = 0

    def is_same(self, ui):
        if ui is None: return False
        if self.name.split('_')[0] == ui.name.split('_'):
            print('UI Image (%s) and UI Image (%s) belong to the UI screenshot' % (self.name, ui.name))
            return True
        if self.has_slide_bar != ui.has_slide_bar:
            # print('UI Image (%s) is different with UI Image (%s)' % (self.name, ui.name))
            return False
        if len(self.compos) != len(ui.compos):
            # print('UI Image (%s) is different with UI Image (%s)' % (self.name, ui.name))
            return False
        for i, compo1 in enumerate(self.compos):
            compo2 = ui.compos[i]
            col_min_a, row_min_a, col_max_a, row_max_a = compo1.bbox.put_bbox()
            col_min_b, row_min_b, col_max_b, row_max_b = compo2.bbox.put_bbox()
            if col_min_a != col_min_b or row_min_a != row_min_b or col_max_a != col_max_b or row_max_a != row_max_b \
                    or (compo1.area != compo2.area) or (self.img[row_min_a:row_max_a, col_min_a:col_max_a]- ui.img[row_min_b:row_max_b, col_min_b:col_max_b]).any() != 0:
                # print('UI Image (%s) is different with UI Image (%s)' % (self.name, ui.name))
                return False
        print('UI Image (%s) is same with UI Image (%s)' % (self.name, ui.name))
        return True

    def find_slide_bar(self):
        slide_bars = []
        for i, compo in enumerate(self.compos):
            if compo.category == 'SlideBar':
                slide_bars.append((compo, i))
        return slide_bars

    def update_next_button(self):
        self.compo_cursor += 1

    def save_connection(self, compo_id, ui_name):
        self.sub_ui_images[compo_id] = ui_name

    def end_search(self):
        return self.compo_cursor == len(self.compos)

    def get_next_button(self):
        compo = self.compos[self.compo_cursor]
        self.update_next_button()
        return compo

    def get_first_button(self):
        compo = self.compos[0]
        self.update_next_button()
        return compo

    def update_buttons(self):
        pass