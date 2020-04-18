#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt

#change ratio
gnss_ratio = 0.5

gnss_error = [[2.5,2.5,10,10,1,2]]
gnss_rnd_error = 0.06
odo_sys_error, odo_init_error, odo_rnd_error = 0.001, 0.1, 0.01

drive_pattern = [[1,0,5],[0,np.pi/2/3,3],[1,0,5],[0,np.pi/2/3,3],[1,0,5],[0,np.pi/2/3,3],[1,0,5],[0,np.pi/2/3,3]]

dt = 0.1
arrow = 1
x = y = the = 0
X, Y, THE = [0], [0], [0]
odo_x, odo_y, odo_the = 0, 0, odo_init_error
ODO_TIME, ODO_X, ODO_Y, ODO_THE = [0], [0], [0], [odo_the]
ODO_DISP_X, ODO_DISP_Y, ODO_DISP_U, ODO_DISP_V = [0], [0], [arrow * math.cos(odo_the)], [arrow * math.sin(odo_the)]
odo_last_time = 0
GNSS_TIME, GNSS_X, GNSS_Y = [0], [0], [0]
gnss_last_time = 0
time = 0
for p in drive_pattern:
	for t in np.arange(0, p[2], dt):
		time += dt
		x, y, the = x + p[0] * math.cos(the) * dt, y + p[0] * math.sin(the) * dt, the + p[1] * dt
		X, Y = X + [x], Y + [y]

		odo_x, odo_y = odo_x + p[0] * math.cos(odo_the) * dt, odo_y + p[0] * math.sin(odo_the) * dt
		odo_the = odo_the + p[1] * dt + odo_sys_error + np.random.normal(0, odo_rnd_error)
		ODO_TIME, ODO_X, ODO_Y, ODO_THE = ODO_TIME + [time], ODO_X + [odo_x], ODO_Y + [odo_y], ODO_THE + [odo_the] 
		if time - odo_last_time >= 1.0:
			odo_last_time = time
			ODO_DISP_X, ODO_DISP_Y = ODO_DISP_X + [odo_x], ODO_DISP_Y + [odo_y]
			ODO_DISP_U, ODO_DISP_V = ODO_DISP_U + [arrow * math.cos(odo_the)], ODO_DISP_V + [arrow * math.sin(odo_the)]
		if time - gnss_last_time >= 1.0:
			gnss_last_time = time
			gnss_error_x = gnss_error_y = 0
			for g in gnss_error:
				if g[0] <= x <= g[2] and g[1] <= y <= g[3]:
					gnss_error_x, gnss_error_y = g[4], g[5]
			GNSS_TIME += [time]
			GNSS_X += [x + gnss_error_x + np.random.normal(0,gnss_rnd_error)]
			GNSS_Y += [y + gnss_error_y + np.random.normal(0,gnss_rnd_error)]

est_x, est_y, est_the = 0, 0, odo_init_error
EST_X, EST_Y, EST_THE = [], [], []
odo_n, gnss_n = 1, 1
last_the = est_the
for t in np.arange(0, time, dt):
	if t >= ODO_TIME[odo_n]:
		dx, dy, dthe = ODO_X[odo_n]-ODO_X[odo_n-1], ODO_Y[odo_n]-ODO_Y[odo_n-1], ODO_THE[odo_n] - ODO_THE[odo_n-1]
		est_delta = math.sqrt(dx**2+dy**2)
		est_x, est_y = est_x + est_delta * math.cos(est_the), est_y + est_delta * math.sin(est_the)
		est_the += dthe
		odo_n += 1
		EST_X, EST_Y, EST_THE = EST_X + [est_x], EST_Y + [est_y], EST_THE + [est_the]
	if gnss_n < len(GNSS_TIME) and t >= GNSS_TIME[gnss_n]:
#		if GNSS_X[gnss_n] < 5.5 and GNSS_Y[gnss_n] < 5.5:
		if True:
			err_x, err_y = GNSS_X[gnss_n] - est_x, GNSS_Y[gnss_n] - est_y		
			error_the = 0
			if 	gnss_n + 1 < len(GNSS_TIME) and GNSS_X[gnss_n+1] < 5.5 and GNSS_Y[gnss_n+1] < 5.5:
				dx, dy = GNSS_X[gnss_n+1] - GNSS_X[gnss_n], GNSS_Y[gnss_n+1] - GNSS_Y[gnss_n]		
				the = math.atan2(dy, dx) 
				error_the = the - est_the if math.sqrt(dx**2 + dy**2) > 0.2 else 0
			error_the = (error_the + math.pi) % (2.0 * math.pi) - math.pi
			est_x, est_y, est_the = est_x + gnss_ratio * err_x, est_y + gnss_ratio * err_y, est_the + gnss_ratio * error_the
		gnss_n += 1

plt.plot(X, Y)
plt.plot(GNSS_X, GNSS_Y, marker='*')
plt.quiver(ODO_DISP_X, ODO_DISP_Y, ODO_DISP_U, ODO_DISP_V, angles='xy',scale_units='xy',scale=2)
plt.gca().set_aspect('equal', adjustable='box')
plt.plot(EST_X, EST_Y, marker='.')
plt.show()

