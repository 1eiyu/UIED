import PySimpleGUI as sg
import cv2
import numpy as np
import base64


def show_points(graph,pointlist):
    for point in pointlist:
        graph.bring_figure_to_front(point)


def select_corner_gui(img = None,camera = None,mtx=None,dist=None):
    newcameramtx = None
    if not(camera == None):
        cap = camera
        ret, frame = cap.read()
        height, width = frame.shape[:2]
        newcameramtx,roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (width,height), 1, (width,height))
        x1, y1, w, h = roi
        height,width = h,w
    elif img is not None:
        height, width = img.shape[:2]
    
    #point select window
    layout = [[sg.Graph(
        canvas_size=(width, height),
        graph_bottom_left=(0, 0),
        graph_top_right=(width, height),
        key="-GRAPH-",
        change_submits=True,  # mouse click events
        background_color='black',
        drag_submits=True), ],
        [sg.Text(text="select first point",key='info', size=(60, 1)),sg.Button("delete"),sg.VSep(),sg.Button("confirm")]]

    window = sg.Window("Corner Selection", layout, finalize=True,resizable=True)

    graph = window["-GRAPH-"]

    if img is not None:
        #https://jdhao.github.io/2020/03/17/base64_opencv_pil_image_conversion/
        _, im_arr = cv2.imencode('.png', img)  # im_arr: image in Numpy one-dim array format.
        im_bytes = im_arr.tobytes()
        im_b64 = base64.b64encode(im_bytes)

        graph.draw_image(data = im_b64, location=(0,height))


    prior_point = None
    pointlist = []
    coordinatelist = []
    polygon = None
    drawnimage = None

    while True:
        event, values = window.read(timeout=50)
        if True:
            if camera is not None:
                ret, frame = cap.read()
                #if not(newcameramtx == None):
                frame = cv2.undistort(frame, mtx, dist, None, newcameramtx)
                frame = frame[y1:y1+h, x1:x1+w]
                imgbytes = cv2.imencode('.png', frame)[1].tobytes()
                im_b64 = base64.b64encode(imgbytes)
                if not(drawnimage == None):
                    graph.delete_figure(drawnimage)
                drawnimage = graph.draw_image(data = im_b64, location=(0,height))
                show_points(graph,pointlist)


        #if event == sg.WIN_CLOSED:
            #break  # exit

        if event == "-GRAPH-":  # if there's a "Graph" event, then it's a mouse
            #print(event,values)
            x, y = values["-GRAPH-"]
            
            if prior_point:
                graph.delete_figure(prior_point)
            if None not in (x, y):
                prior_point = graph.draw_point((x,y),size=5,color='red')
            
        
        elif event.endswith('+UP'):
            if len(coordinatelist) >= 4:
                graph.delete_figure(prior_point)
                continue
            #add the coordinates and points to save
            coordinatelist.append((x,height-y))
            point = graph.draw_point((x,y),size=5,color='red')
            pointlist.append(point)
            #update the info
            if len(coordinatelist)<4:
                info = window["info"]
                string = "select the" + str(len(coordinatelist)+1) + "point"
                info.update(value=string)
            elif len(coordinatelist) == 4:
                info = window["info"]
                info.update(value="coordinates defined")
                #draw polygon and change the coordinates of the list
                #polygon = graph.draw_polygon({(x,height-y) for (x,y) in coordinatelist},line_color = "green")
            
            print(coordinatelist)
        
        
        elif event == "delete":
            if len(coordinatelist) > 0:
                #if len(coordinatelist) == 4:
                    #graph.delete_figure(polygon)
                coordinatelist.pop()
                todelete = pointlist[-1]
                pointlist.pop()
                graph.delete_figure(todelete)
                if prior_point:
                    graph.delete_figure(prior_point)
            print("delete")
            #print(pointlist)
        elif event == "confirm":
            break
        elif event == "__TIMEOUT__":
            continue
        else:
            print("unhandled event", event, values)
        
    window.close()
    return coordinatelist
