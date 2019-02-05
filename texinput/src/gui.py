import PySimpleGUI as sg # pip install PySimpleGUI
#import rospy
#import roslib

#pub_speed = rospy.Publisher("/localization/desired_speed", Int16, queue_size=1, latch=True)
#pub_look_ahead = rospy.Publisher("/localization/look_ahead", Int16, queue_size=1, latch=True)
#pub_look_ahead_curve = rospy.Publisher("/localization/look_ahead_curve", Int16, queue_size=1, latch=True)

layout = [ [sg.InputText(key='s'), sg.Button('Desired Speed'), sg.Txt('', key='so')],
           [sg.InputText(key='l'), sg.Button('Line Lookahead'), sg.Txt('', key='lo')],
           [sg.InputText(key='c'), sg.Button('Curve lookahead'), sg.Txt('', key='co')],
           [sg.Cancel()] ]

window = sg.Window('model car GUI').Layout(layout)

while (True):
    event, values = window.Read()
    if event is 'Desired Speed' and str.isdigit(values['s']):
        window.FindElement('so').Update(values['s'])
        #pub_speed.publish(Int16(values[s]))
    if event is 'Line Lookahead' and str.isdigit(values['l']):
        window.FindElement('lo').Update(values['l'])
        #pub_look_ahead_curve.publish(Int16(values[l]))
    if event is 'Curve lookahead' and str.isdigit(values['c']):
        window.FindElement('co').Update(values['c'])
        #pub_look_ahead.publish(Int16(values[c]))
    if event == 'Cancel':
        break

window.Close()
exit()