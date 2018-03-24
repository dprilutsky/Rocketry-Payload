from kivy.app import App
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, Property
from kivy.core.window import Window
from kivy.garden.graph import Graph, MeshLinePlot
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
import serial
from re import findall

from math import sin # for testing purposes.
com = None
try:
    com = serial.Serial('COM4', timeout=0.5)
except:
    "Error: no serial communication detected"
    com = None
i=0
keys = '0123456789rspalbv' #valid keyboard commands
class DataLine: # a line of data
    def __init__(self, dataPoints):
        ##rtb not implemented yet
        self.accelX   = dataPoints[0]
        self.accelY   = dataPoints[1]
        self.accelZ   = dataPoints[2]
        self.roll     = dataPoints[3]
        self.pitch    = dataPoints[4]
        self.heading  = dataPoints[5]
        self.baroAlt  = dataPoints[6]
        self.GPSalt   = dataPoints[7]
        self.GPSspeed = dataPoints[8]
        self.lat      = dataPoints[9]
        self.lon      = dataPoints[10]
        self.time     = dataPoints[11]
        
def transmit(data):
    #Send to serial
    if (com != None):
        com.write(data.encode('ascii'))
    else:
        print("No xbee communication. transmission shown below.")
    print(data.encode('ascii'))
    

    
def displayCommaSep(ls):
    '''returns the given list of numbers as a comma separated list'''
    return ", ".join(str(x) for x in ls)

def readAndParseData():
    # so there's gonna be some time limit to see if the rocket is transmitting or not.
    # then it's gonna keep collecting data from the serial until encountering '&'
    # then it'll validate that the data isn't corrupted and return an object with the data
    dataString=""
    #dataString="*#xaccel#,#yaccel#,#zaccel#,#roll#,#pitch#,#heading#,#baraltitude#,#gpsaltitude#,#gpsspeed#,#lat#,#lon#,#seconds#&"
    if (com!=None):
        while True:
            nextChar=com.read()
            dataString+=nextChar.decode("utf-8")
            if(nextChar == b'&' or nextChar == b''):
                break
        
        
    else:
        data=None
        print("no xbee communication")
    dataList = findall("#(.*?)#", dataString)
    print(dataList)
    if (len(dataList) == 12):
        return DataLine(dataList)
    else:
        return None
    
def updateGraph(data):
    #https://stackoverflow.com/questions/22831879/how-to-create-real-time-graph-in-kivy
    return

class DataWidget(Widget):
    accel = Property('NaN')
    accelMax = Property('NaN')
    accelMaxes = (0, 0, 0)
    altitude = NumericProperty(0)
    altitudeMax = NumericProperty(0)
    coordinates = Property("unavailable")
    gpsSpeed = NumericProperty(0)
    gpsSpeedMax = NumericProperty(0)
    status = Property('unavailable')
##    graph_test = Graph(xlabel='X', ylabel='Y', x_ticks_minor=5,
##    x_ticks_major=25, y_ticks_major=1,
##    y_grid_label=True, x_grid_label=True, padding=5,
##    x_grid=True, y_grid=True, xmin=-0, xmax=100, ymin=-1, ymax=1)

    
    # keyboard stuff
    def __init__(self, **kwargs):
        super(DataWidget, self).__init__(**kwargs)
        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        self._keyboard.bind(on_key_down=self._on_keyboard_down)
        
        #graph stuff
        self.altigraph    = self.ids.graph_alt
        ##self.graph.x_ticks_major = 25
##        self.graph.xlabel='Time'
##        self.graph.ylabel='Value'
##        self.graph.y_ticks_major= 1
##        self.graph.y_grid_label= True
##        self.graph.x_grid_label= True
##        self.graph.padding= 5
##        self.graph.xmin=0
##        self.graph.xmax=100
##        self.graph.ymin=-1
##        self.graph.ymax:1
        
        self.altiplot = MeshLinePlot(color=[1, 0, 0, 1])
        #self.plot.points = [(x, sin(x / 10.)) for x in range(0, 101)]
        self.altigraph.add_plot(self.altiplot)

        self.accelgraph = self.ids.graph_accel
        self.accelplotx = MeshLinePlot(color=[1, 0, 0, 1])
        self.accelgraph.add_plot(self.accelplotx)
        self.accelploty = MeshLinePlot(color=[1, 1, 0, 1])
        self.accelgraph.add_plot(self.accelploty)
        self.accelplotz = MeshLinePlot(color=[0, 1, 0, 1])
        self.accelgraph.add_plot(self.accelplotz)

        
        #self.ids.buttons.add_widget(TextInput(multiline=False))
    
    def _keyboard_closed(self):
        self._keyboard.unbind(on_key_down=self._on_keyboard_down)
        self._keyboard = None

    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        k = keycode[1]
        if k in keys:
            transmit(k)
        return True
    def update(self, dt):
        data = readAndParseData()
        #print(data)
        #self.accel = data.accelX + ", " + data.accelY + ", " + data.accelZ
        ## etc.
        #updateGraph(data)
        global i
        if (data != None):
            self.accel = displayCommaSep((data.accelX, data.accelY, data.accelZ))
            if abs(data.accelX) > abs(accelMaxes[0]):
                accelMaxes[0] = data.accelX
            if abs(data.accelY) > abs(accelMaxes[1]):
                accelMaxes[1] = data.accelY
            if abs(data.accelZ) > abs(accelMaxes[2]):
                accelMaxes[2] = data.accelZ
            self.accelMax = displayCommaSep(accelMaxes)
            self.accelplotx.points.append((data.time, data.accelX))
            self.accelploty.points.append((data.time, data.accelY))
            self.accelplotz.points.append((data.time, data.accelZ))
            self.altitude = data.baroAlt
            self.altitudeMax = max(self.altitudeMax, data.baroAlt)
            self.coordinates = displayCommaSep([i, -i])
            self.gpsSpeed = data.GPSspeed
            self.gpsSpeedMax = max(self.gpsSpeedMax, data.GPSspeed)
            self.status = '1rtn'
            #self.plot.points = [(x, sin(x / 10.)) for x in range(0, 101)]
            pointx = float(data.time)
            pointy = float(data.baroAlt)
            self.altiplot.points.append((pointx, pointy))
            if (pointx > self.altigraph.xmax):
                self.altigraph.xmax*=1.1
                #self.graph.x_ticks_major = self.graph.xmax/5
            if (pointy > self.altigraph.ymax):
                self.altigraph.ymax*=1.1
            if (pointy < self.altigraph.ymin):
                self.altigraph.ymin*=1.1

            if (float(data.accelX) > self.accelgraph.xmax):
                self.accelgraph.xmax *= 1.1
            #if (i/10 + 1 > self.accelgraph.ymax):
             #   self.accelgraph.ymax *= 1.1
        i+=1
        pass


class RocketApp(App):
    
    popupContent = BoxLayout(orientation='vertical')
    popupButton = Button(text='submit')
    popupInput=TextInput(text='0', multiline=False, id="theTextInput")
    def onLaunchSubmit(instance):
        print(instance)
        print(widget.ids.theTextInput)
    #popupInput.bind(on_text_validate=onLaunchSubmit)
    popupButton.bind(on_release=onLaunchSubmit)
    popupContent.add_widget(popupInput)
    popupContent.add_widget(popupButton)
    #popupButton.bind(on_release=sendLaunchNumber)
    popup = Popup(title='Set launch number',
            content=popupContent,
            size_hint=(None, None), size=(400, 400))
    
    def build(self):
        #
        self.root = Builder.load_file('rocket.kv')
        widget = DataWidget()
        
        i = 0
        Clock.schedule_interval(widget.update, 1.0/10.0)
        return widget
    def pressButton(idontcare, text):
        transmit(text)

if __name__ == '__main__':
    RocketApp().run()
