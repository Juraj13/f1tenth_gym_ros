from matplotlib.widgets import Button
import matplotlib as mpl
from matplotlib import pyplot as plt
# import scipy.interpolate as inter
import numpy as np

# my classes
from TrajectoryPlanner import TrajectoryPlanner

# !!!!! remember trasformation(y = 600-y)   !!!!!!!!!!!!!!!!!

trajectory_points = np.empty((0,2))
trajectory_circles = []

trajectory_planner = None

#spline fit - nis ne radi samo kao bookmark
# spline = inter.InterpolatedUnivariateSpline (x, yvals)

#figure.subplot.right
mpl.rcParams['figure.subplot.right'] = 0.8

#set up a plot
fig,axes = plt.subplots(1,1,figsize=(9.0,8.0),sharex=True)
plotline = None


pind = None #active point
epsilon = 5 #max pixel distance

def update():
    'draw all trajectory points'

    global trajectory_points
    global trajectory_circles
    global plotline
    for i in range(len(trajectory_points)):
        x = trajectory_points[i][0]
        y = trajectory_points[i][1]
        if len(trajectory_circles) == i:
            trajectory_circles.append( plt.Circle((x,y), epsilon, color='r'))
            axes.add_artist(trajectory_circles[i])
        else:
            trajectory_circles[i].center = (x,y)
    
    trajecory = trajectory_planner.get_trajectory()
    x = trajecory[:,0]
    y = trajecory[:,1]
    while plotline:
        plotline.pop(0).remove()
    plotline = axes.plot(x,600-y)

    fig.canvas.draw_idle()


def done(event):
    global trajectory_points
    global trajectory_planner
    xy = trajectory_points[0]
    trajectory_planner.add_trajectory_point([xy[0], 600-xy[1]])
    update()

def save(event):
    global trajectory_planner
    trajectory = trajectory_planner.get_trajectory()
    with open('trajectory.csv', 'w') as file:
        for i in range(len(trajectory)):
            file.write("%f, %f\n" % (trajectory[i][0], trajectory[i][1]))
    print("Saved")


def button_press_callback(event):
    'whenever a mouse button is pressed'
    global pind
    if event.inaxes is None or event.inaxes != axes:
        return
    if event.button != 1:
        return
    pind = get_ind_under_point(event)
    # print(pind)
    if pind is None:
        add_trajectory_point(event)
        update()

def button_release_callback(event):
    'whenever a mouse button is released'
    global pind
    if event.button != 1:
        return
    if pind is not None:
        trajectory_planner.update_trajectory_point(pind, [event.xdata, 600-event.ydata])
    pind = None
    update()

def add_trajectory_point(event):
    'add new point to trajectory points'

    global trajectory_points
    global trajectory_planner
    xy = [event.xdata, event.ydata]
    trajectory_points = np.append(trajectory_points, [xy], axis=0)
    trajectory_planner.add_trajectory_point([xy[0], 600-xy[1]])

def get_ind_under_point(event):
    'get the index of the vertex under point if within epsilon tolerance'

    global trajectory_points
    if trajectory_points.size == 0:
        return None

    # display coords
    # print('display x is: {0}; display y is: {1}'.format(event.x,event.y))
    # print('display x is: {0}; display y is: {1}'.format(event.xdata,event.ydata))

    x = trajectory_points[:, 0]
    y = trajectory_points[:, 1]
    d = np.hypot(x - event.xdata, y - event.ydata)
    indseq, = np.nonzero(d == d.min())
    ind = indseq[0]

    #print(d[ind])
    if d[ind] >= epsilon:
        ind = None
    
    #print(ind)
    return ind

def motion_notify_callback(event):
    'on mouse movement'
    global trajectory_points
    if pind is None:
        return
    if event.inaxes is None:
        return
    if event.button != 1:
        return
    
    #update trajectory points
    # print('motion x: {0}; y: {1}'.format(event.xdata,event.ydata))
    trajectory_points[pind] = [event.xdata, event.ydata]
    update()


if __name__ == '__main__':
    pgm = plt.imread('/home/ivan/catkin_ws/src/f1tenth_gym_ros/map.pgm')
    plt.imshow(pgm)

    trajectory_planner = TrajectoryPlanner(np.empty((0,2)), 0, 100)

    axdon = plt.axes([0.84, 0.8-((10)*0.05), 0.12, 0.02])
    bres = Button(axdon, 'Done')
    bres.on_clicked(done)

    axsav = plt.axes([0.84, 0.8-((8)*0.05), 0.12, 0.02])
    bsav = Button(axsav, 'Save')
    bsav.on_clicked(save)
    

    fig.canvas.mpl_connect('button_press_event', button_press_callback)
    fig.canvas.mpl_connect('button_release_event', button_release_callback)
    fig.canvas.mpl_connect('motion_notify_event', motion_notify_callback)

    plt.show()