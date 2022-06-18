
from fileinput import filename
from sqlite3 import Time
import time
from tokenize import String
import PySimpleGUI as sg
import cv2
import base64
import numpy as np
from Functions import img_functions3

class UI_Skript_Recorder():
    
    statedict = {
        "Press Button" : 0,
        "Drag": 1,
        "Double Tap": 2,
        "Trigger": 3
    }

    def __init__(self,camera,coordinatelist=[],mtx=None,dist=None,ctl_interface = None):
        self.state = 0
        self.ctl_interface = ctl_interface
        self.mtx = mtx
        self.dist = dist
        self.coordinatelist = coordinatelist
        self.camera = camera
        self.newcameramtx = None
        self.x1, self.y1, self.w, self.h = None, None, None, None
        if len(coordinatelist) == 4:
            print("wrap perspective mode")
            from Functions import img_functions3
            ret, self.frame = camera.read()
            self.newcameramtx,roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (self.frame.shape[1],self.frame.shape[0]), 1, (self.frame.shape[1],self.frame.shape[0]))
            self.x1, self.y1, self.w, self.h = roi
            self.frame = cv2.undistort(self.frame, self.mtx, self.dist, None, self.newcameramtx)
            self.frame = self.frame[self.y1:self.y1+self.h, self.x1:self.x1+self.w]
            result = img_functions3.wrap_perspective(self.frame,coordinatelist)
            self.height = result.shape[0]
            self.width = result.shape[1]
            print(result.shape)
        else:
            self.height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)

        
        self.elementlist = []
        
        self.layout = self.buildLayout(image_width=self.width,image_height=self.height)
        self.window = sg.Window("Skript Recorder",self.layout,finalize=False,resizable=True)
        self.graph = self.window["-GRAPH-"]
        self.rectangle = None
        self.prev_rectangle = None
        self.point1 = None
        self.command = None
        
        self.eventloop()


    def buildLayout(self,image_width = 1280, image_height = 720 ):
        if self.ctl_interface == None:
            execute_button_enable = False
        else:
            execute_button_enable = True
        layout_right = [[sg.InputText(key="file_open",enable_events=True,visible=False),sg.FileBrowse() , sg.InputText(key="file_save_as_input",enable_events=True,visible=False),sg.FileSaveAs(enable_events=True,)],[sg.Listbox(self.elementlist,size=(24,int(image_height/17)-2),key="-ListBox-",enable_events=True) ],[sg.Button("Add",disabled=True),
            sg.Button("Delete")],[sg.Button("Play"),sg.Button("Execute",visible=execute_button_enable)]]


        layout_right = sg.Column(layout_right)
        layout_botom_default = [sg.Combo(["Press Button","Drag","Double Tap","Trigger"],readonly=True,enable_events=True),
            sg.Combo(["External","Time"],enable_events=True,visible=False,readonly=True,key="-TrigerCombo-"),sg.Input(key="-TimeInput-",visible=False,size=(1,10)),]
            #]
        #layout_extra = [sg.Button("test",visible=False,key="-TrigerCombo-"),]
        
        
        layout = [
            [sg.Graph(
            canvas_size=(image_width, image_height),
            graph_bottom_left=(0, 0),
            graph_top_right=(image_width, image_height),
            key="-GRAPH-",
            change_submits=True,  # mouse click events
            background_color='black',
            drag_submits=True), layout_right ], 
            layout_botom_default,
            #layout_extra
            ]

        return layout


    def handlegraph(self,event,values):
        if self.state == self.statedict["Press Button"]:
            self.handleButtonDrag(event,values)
        elif self.state == self.statedict["Drag"]:
            self.handleDrag(event,values)
        elif self.state == self.statedict["Trigger"]:
            print("Trigger mode")
        elif self.state == self.statedict["Double Tap"]:
            self.handleButtonDrag(event,values,pre_command="DoubleTap: ")
        
    def handleDrag(self,event,values):
        if self.point1 == None:
            self.point1 = (values["-GRAPH-"])
            if not(self.prev_rectangle == None): self.graph.delete_figure(self.prev_rectangle)
        else:
            if not(self.rectangle == None): self.graph.delete_figure(self.rectangle)
            self.rectangle = self.graph.DrawLine(point_from=self.point1,point_to=(values["-GRAPH-"]),color="red")
        if event.endswith('+UP'):
            x,y  = values["-GRAPH-"]
            self.command = "Drag: " + str(self.point1[0])+ " " + str(self.height-self.point1[1]) + " - " + str(x) + " " + str(self.height-y)
            print(self.command)

            self.point1 = None
            self.prev_rectangle = self.rectangle
            self.rectangle = None
            self.window["Add"].update(disabled=False)

    def handleButtonDrag(self,event,values,pre_command="Click: "):
        if self.point1 == None:
            self.point1 = (values["-GRAPH-"])
            if not(self.prev_rectangle == None): self.graph.delete_figure(self.prev_rectangle)
        else:
            if not(self.rectangle == None): self.graph.delete_figure(self.rectangle)
            self.rectangle = self.graph.DrawRectangle(top_left=self.point1,bottom_right=(values["-GRAPH-"]),line_color="red")
        if event.endswith('+UP'):
            
            x,y  = values["-GRAPH-"]
            self.command = pre_command + str((x + self.point1[0])/2) + " - " + str(self.height-(y+self.point1[1])/2)
            print(self.command)

            self.point1 = None
            self.prev_rectangle = self.rectangle
            self.rectangle = None
            self.window["Add"].update(disabled=False)
  

    def switch_mode(self,values):
        if(values[0] in self.statedict):
            self.state = self.statedict[values[0]]
        else:
            return
        if not(self.prev_rectangle==None):
                self.graph.delete_figure(self.prev_rectangle)
        if self.state == self.statedict["Trigger"]:
            self.window["Add"].update(disabled=False)
            self.window["-TrigerCombo-"].update(visible=True)
        else:
            self.window["-TrigerCombo-"].update(visible=False)
            self.window["-TimeInput-"].update(visible=False)
            self.window["Add"].update(disabled=True)

    def execute_command(self):
        indizes = self.window["-ListBox-"].get_indexes()
        if len(indizes)>0:
            command = self.elementlist[indizes[0]]
            commandsplit = command.split(" ")
            if commandsplit[0]=="Click:":
                print("clicking:")
                x_rel = float(commandsplit[1])/float(self.width)
                y_rel = float(commandsplit[3])/float(self.height)
                
                self.ctl_interface.touch_point_relative(x_rel,y_rel)
            elif commandsplit[0]=="DoubleTap:":
                print("DoubleTap:")
                x_rel = float(commandsplit[1])/float(self.width)
                y_rel = float(commandsplit[3])/float(self.height)
                
                self.ctl_interface.touch_point_relative(x_rel,y_rel)
                self.ctl_interface.touch_point_relative(x_rel,y_rel)
            elif commandsplit[0] == "Drag:":
                print(commandsplit)
                x1_rel = float(commandsplit[1])/float(self.width)
                y1_rel = float(commandsplit[2])/float(self.height)
                x2_rel = float(commandsplit[4])/float(self.width)
                y2_rel = float(commandsplit[5])/float(self.height)
                self.ctl_interface.slide_relative(x1_rel,y1_rel,x2_rel,y2_rel)
            elif commandsplit[0] == "Trigger:":
                if commandsplit[1] == "Time":
                    duration = int(commandsplit[2])
                    self.ctl_interface.time_trigger(duration)
                elif commandsplit[1] == "External":
                    self.ctl_interface.wait_for_trigger()

    def add_command(self):
        if self.state == self.statedict["Trigger"]:
            trigger_type = self.window["-TrigerCombo-"].get()
            if trigger_type == "External":
                self.command = "Trigger: External"
            elif trigger_type == "Time":
                timestring = self.window["-TimeInput-"].get()
                try:
                    print(int(timestring))
                except:
                    return
                self.command = "Trigger: Time " + timestring
            
        self.elementlist.append(self.command)
        self.graph.delete_figure(self.prev_rectangle)
        self.prev_rectangle = None
        self.window["Add"].update(disabled=True)
        self.window["-ListBox-"].update(values= self.elementlist)
        self.window["-ListBox-"].set_value(values = [self.elementlist[-1]])

    def delete_command(self):
        todelete = self.window["-ListBox-"].get_indexes()
        for index in todelete:
            del self.elementlist[index]
        self.window["-ListBox-"].update(values= self.elementlist)
        if len(self.elementlist)>1:
            self.window["-ListBox-"].set_value(values = [self.elementlist[-1]])

    
    def draw_camera_image(self):
        if self.point1 == None:
            ret, self.frame = self.camera.read()

            if len(self.coordinatelist)==4:
                self.frame = cv2.undistort(self.frame, self.mtx, self.dist, None, self.newcameramtx)
                self.frame = self.frame[self.y1:self.y1+self.h, self.x1:self.x1+self.w]
                self.frame = img_functions3.wrap_perspective(self.frame,self.coordinatelist)
            imgbytes = cv2.imencode('.png', self.frame)[1].tobytes()
            im_b64 = base64.b64encode(imgbytes)
            if not(self.drawnimage == None):
                self.graph.delete_figure(self.drawnimage)
            self.drawnimage = self.graph.draw_image(data = im_b64, location=(0,self.height))
            if not(self.prev_rectangle==None):
                self.graph.bring_figure_to_front(self.prev_rectangle)

    def play_script(self):
        self.ctl_interface.clear_pose_list()
        values = self.window["-ListBox-"].get_list_values()
        for i,value in enumerate(values):
            print(i)
            print(value)
            #self.window["-ListBox-"].set_value(values[i])
            self.window["-ListBox-"].set_value(values = [value])
            self.command = value
            self.execute_command()
        self.command = None

    def open_file(self,file):
        res = np.loadtxt(file,dtype=str,delimiter=",")
        self.elementlist = list(res)
        self.window["-ListBox-"].update(values= self.elementlist)
        self.window["-ListBox-"].set_value(values = [self.elementlist[-1]])

    def save_file(self,file):
        tosave = np.array(self.elementlist)
        np.savetxt(file,tosave,fmt='%s',delimiter=",")

    def eventloop(self):
        self.drawnimage = None
        while True:
            #read event
            looptime = time.time()
            event, values = self.window.read(timeout=100)
            if event == None:
                break
            elif event == 0:
                self.switch_mode(values)
            elif event == "-GRAPH-" or event.endswith("+UP"):
                self.handlegraph(event,values)
            elif event == "Add":
                print("add")
                self.add_command()
            elif event == "Delete":
                self.delete_command()
            elif event == "Execute":
                self.execute_command()
            elif event == "Play":
                self.play_script()
            elif event == "__TIMEOUT__":
                self.draw_camera_image()
            elif event == "-TrigerCombo-":
                if values["-TrigerCombo-"] == "Time":
                    self.window["-TimeInput-"].update(visible=True)
                else:
                    self.window["-TimeInput-"].update(visible=False)
            elif event == "file_save_as_input":
                filename = values["file_save_as_input"]
                if filename:
                    self.save_file(filename)
            elif event == "file_open":
                filename = values["file_open"]
                self.open_file(filename)
            else:
                print(event)
                print(values)
            #print((time.time()-looptime))
    
