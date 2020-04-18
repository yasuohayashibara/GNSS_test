#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

gnss_error = [[2.5,2.5,10,10,1,2]]
gnss_rnd_error = 0.002
odo_sys_error, odo_init_error, odo_rnd_error = 0.01, 0.1, 0.001

drive_pattern = [[0.5,0,10],[0,np.pi/2/3,3],[0.5,0,10],[0,np.pi/2/3,3],[0.5,0,10],[0,np.pi/2/3,3],[0.5,0,10],[0,np.pi/2/3,3]]

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

# simulation for odometry and GNSS
for p in drive_pattern:
	for t in np.arange(0, p[2], dt):
		time += dt
		x, y, the = x + p[0] * math.cos(the) * dt, y + p[0] * math.sin(the) * dt, the + p[1] * dt
		X, Y = X + [x], Y + [y]

		odo_x, odo_y = odo_x + p[0] * math.cos(odo_the) * dt, odo_y + p[0] * math.sin(odo_the) * dt
		odo_the = odo_the + p[1] * dt + odo_sys_error * dt + np.random.normal(0, odo_rnd_error)
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

# detect relable GNSS data
GNSS_SCORE = [0]
odo_n, gnss_n = 1, 1
prev_odo_n = [0, 0]
for t in np.arange(0, time, dt):
	if t >= ODO_TIME[odo_n]:
		odo_n += 1
	if gnss_n < len(GNSS_TIME) and t >= GNSS_TIME[gnss_n]:
		if gnss_n > 1:
			odo0_dx, odo0_dy = ODO_X[prev_odo_n[0]] - ODO_X[prev_odo_n[1]], ODO_Y[prev_odo_n[0]] - ODO_Y[prev_odo_n[1]]
			odo1_dx, odo1_dy = ODO_X[odo_n]         - ODO_X[prev_odo_n[0]], ODO_Y[odo_n]         - ODO_Y[prev_odo_n[0]]
			odo_delta_the = ODO_THE[prev_odo_n[0]] - ODO_THE[prev_odo_n[1]]
			gnss0_dx, gnss0_dy = GNSS_X[gnss_n-1] - GNSS_X[gnss_n-2], GNSS_Y[gnss_n-1] - GNSS_Y[gnss_n-2]
			gnss1_dx, gnss1_dy = GNSS_X[gnss_n  ] - GNSS_X[gnss_n-1], GNSS_Y[gnss_n  ] - GNSS_Y[gnss_n-1]
			gnss_delta_the = math.atan2(gnss1_dy, gnss1_dx) - math.atan2(gnss0_dy, gnss0_dx)
			error0 = math.sqrt(odo0_dx**2+odo0_dy**2) - math.sqrt(gnss0_dx**2+gnss0_dy**2)
			error1 = math.sqrt(odo1_dx**2+odo1_dy**2) - math.sqrt(gnss1_dx**2+gnss1_dy**2)
			error_the = 0
			if math.sqrt(gnss0_dx**2+gnss0_dy**2) > 0.2 and math.sqrt(gnss1_dx**2+gnss1_dy**2) > 0.2:
				error_the = (gnss_delta_the - odo_delta_the + math.pi) % (2.0 * math.pi) - math.pi
			GNSS_SCORE += [1.0/(1.0+math.fabs(error0)+math.fabs(error1)+math.fabs(error_the))]
		prev_odo_n[1] = prev_odo_n[0]
		prev_odo_n[0] = odo_n
		gnss_n += 1
GNSS_SCORE += [0]

GNSS_FI_SCORE, GNSS_BI_SCORE, GNSS_I_SCORE = [0]*len(GNSS_SCORE),[0]*len(GNSS_SCORE),[0]*len(GNSS_SCORE)
gnss_min_score = 0.5
for i in range(len(GNSS_SCORE)):
	if GNSS_SCORE[i] > gnss_min_score and i > 0:
		GNSS_FI_SCORE[i] = GNSS_FI_SCORE[i-1] + GNSS_SCORE[i]
	else:
		GNSS_FI_SCORE[i] = GNSS_SCORE[i]
	j = len(GNSS_SCORE) - 1 - i
	if GNSS_SCORE[j] > gnss_min_score and j < len(GNSS_SCORE) - 1:
		GNSS_BI_SCORE[j] = GNSS_BI_SCORE[j+1] + GNSS_SCORE[j]
	else:
		GNSS_BI_SCORE[j] = GNSS_SCORE[j]

gnss_max_i_score = gnss_max_i_score_n = 0
for i in range(len(GNSS_SCORE)):
	GNSS_I_SCORE[i] = min([GNSS_FI_SCORE[i], GNSS_BI_SCORE[i]])
	if GNSS_I_SCORE[i] > gnss_max_i_score:
		gnss_max_i_score = GNSS_I_SCORE[i]
		gnss_max_i_score_n = i


gnss_reliable_n_min, gnss_reliable_n_max = 0, len(GNSS_SCORE)-1
for i in range(gnss_max_i_score_n, len(GNSS_SCORE)):
	if GNSS_I_SCORE[i] < gnss_min_score:
		gnss_reliable_n_max = i - 1
		break
for i in range(gnss_max_i_score_n, 0, -1):
	if GNSS_I_SCORE[i] < gnss_min_score:
		gnss_reliable_n_min = i + 1
		break

print(gnss_reliable_n_min, gnss_reliable_n_max)

#calculate system error of odometry
dist_error = dist_error_n = angle_err = angle_err_n = 0
#for i in range(gnss_reliable_n_min, gnss_reliable_n_max):
odo_n = gnss_n = 0
prev_odo_n[0] = 0
for t in np.arange(0, time, dt):
	if gnss_n < len(GNSS_TIME) and t >= GNSS_TIME[gnss_n]:
		if gnss_reliable_n_min < gnss_n < gnss_reliable_n_max:
			odo_dx, odo_dy = ODO_X[odo_n] - ODO_X[prev_odo_n[0]], ODO_Y[odo_n] - ODO_Y[prev_odo_n[0]]
			odo_delta_the = ODO_THE[odo_n] - ODO_THE[prev_odo_n[0]]
			gnss0_dx, gnss0_dy = GNSS_X[gnss_n] - GNSS_X[gnss_n-1], GNSS_Y[gnss_n] - GNSS_Y[gnss_n-1]
			gnss1_dx, gnss1_dy = GNSS_X[gnss_n+1] - GNSS_X[gnss_n], GNSS_Y[gnss_n+1] - GNSS_Y[gnss_n]
			gnss_delta_the = math.atan2(gnss1_dy, gnss1_dx) - math.atan2(gnss0_dy, gnss0_dx)
			dt = ODO_TIME[odo_n] - ODO_TIME[prev_odo_n[0]]
			dist_error += (math.sqrt(odo_dx**2+odo_dy**2) - math.sqrt(gnss0_dx**2+gnss0_dy**2))/dt
			dist_error_n += 1
			if math.sqrt(gnss0_dx**2+gnss0_dy**2) > 0.3 and math.sqrt(gnss1_dx**2+gnss1_dy**2) > 0.3:
				angle_err += ((gnss_delta_the - odo_delta_the + math.pi) % (2.0 * math.pi) - math.pi)/dt
				angle_err_n += 1
#				print(((gnss_delta_the - odo_delta_the + math.pi) % (2.0 * math.pi) - math.pi)/dt)
		prev_odo_n[0] = odo_n
		gnss_n += 1
	if t >= ODO_TIME[odo_n]:
		odo_n += 1
dist_error /= dist_error_n
angle_err /= angle_err_n
print(dist_error, angle_err)

#modify odometry
odo_x, odo_y, odo_the = 0, 0, odo_init_error
ODO_MOD_X, ODO_MOD_Y, ODO_MOD_THE = [0], [0], [odo_the]
for i in range(1,len(ODO_TIME)):
	dx, dy, dthe = ODO_X[i]-ODO_X[i-1], ODO_Y[i]-ODO_Y[i-1], ODO_THE[i]-ODO_THE[i-1]
	dt = ODO_TIME[i]-ODO_TIME[i-1] 
	dist = math.sqrt(dx**2+dy**2) + dist_error * dt
	odo_x, odo_y = odo_x + dist * math.cos(odo_the), odo_y + dist * math.sin(odo_the)
	odo_the += dthe + angle_err * dt
	ODO_MOD_X, ODO_MOD_Y, ODO_MOD_THE = ODO_MOD_X+[odo_x], ODO_MOD_Y+[odo_y], ODO_MOD_THE+[odo_the]

#change initial position
init_x, init_y, init_the, init_n = 0, 0, 0, 0
odo_n = gnss_n = 0
for t in np.arange(0, time, dt):
	if gnss_n < len(GNSS_TIME) and t >= GNSS_TIME[gnss_n]:
		if gnss_reliable_n_min < gnss_n < gnss_reliable_n_max:
			gnss_dx, gnss_dy = GNSS_X[gnss_n+1] - GNSS_X[gnss_n], GNSS_Y[gnss_n+1] - GNSS_Y[gnss_n]
			if math.sqrt(gnss_dx**2+gnss_dy**2) > 0.3:			
				gnss_the = math.atan2(gnss_dy, gnss_dx)
				init_the += (gnss_the - ODO_MOD_THE[odo_n] + math.pi) % (2.0 * math.pi) - math.pi
				init_n += 1
		gnss_n += 1
	if t >= ODO_TIME[odo_n]:
		odo_n += 1
init_the /= init_n

odo_n = gnss_n = init_n = 0
for t in np.arange(0, time, dt):
	if gnss_n < len(GNSS_TIME) and t >= GNSS_TIME[gnss_n]:
		if gnss_reliable_n_min < gnss_n < gnss_reliable_n_max:
			odo_x = ODO_MOD_X[odo_n] * math.cos(init_the) - ODO_MOD_Y[odo_n] * math.sin(init_the)
			odo_y = ODO_MOD_X[odo_n] * math.sin(init_the) + ODO_MOD_Y[odo_n] * math.cos(init_the)
			dx, dy = GNSS_X[gnss_n] - odo_x, GNSS_Y[gnss_n] - odo_y
			init_x, init_y = init_x + dx, init_y + dy
			init_n += 1
		gnss_n += 1
	if t >= ODO_TIME[odo_n]:
		odo_n += 1
init_x, init_y = init_x/init_n, init_y/init_n
print(init_the, init_x, init_y)

odo_x, odo_y, odo_the = 0, 0, odo_init_error
ODO_INI_X, ODO_INI_Y, ODO_INI_THE = [0], [0], [odo_the]
for i in range(1,len(ODO_TIME)):
	ODO_INI_X += [ODO_MOD_X[i]*math.cos(init_the)-ODO_MOD_Y[i]*math.sin(init_the)+init_x]
	ODO_INI_Y += [ODO_MOD_X[i]*math.sin(init_the)+ODO_MOD_Y[i]*math.cos(init_the)+init_y]
	ODO_INI_THE += [ODO_MOD_THE[i]+init_the]

# covariance
GNSS_COV = [0.1]*len(GNSS_TIME)
cov = 0.1
for i in range(gnss_reliable_n_min, 0, -1):
	GNSS_COV[i] = cov
	cov += 0.01
cov = 0.1
for i in range(gnss_reliable_n_max, len(GNSS_TIME)):
	GNSS_COV[i] = cov
	cov += 0.01

# reliable
odo_n = gnss_n = 0
GNSS_REL = []
for t in np.arange(0, time, dt):
	if gnss_n < len(GNSS_TIME) and t >= GNSS_TIME[gnss_n]:
		dx, dy = GNSS_X[gnss_n] - ODO_INI_X[odo_n], GNSS_Y[gnss_n] - ODO_INI_Y[odo_n]
		error = math.sqrt(dx**2+dy**2)
		if error < GNSS_COV[gnss_n]:
			GNSS_REL += [True]
		else:
			GNSS_REL += [False]
		gnss_n += 1
	if t >= ODO_TIME[odo_n]:
		odo_n += 1

#merge odometry and GNSS
gnss_ratio = 0.5
est_x, est_y, est_the = 0, 0, odo_init_error + init_the
EST_X, EST_Y, EST_THE = [], [], []
odo_n, gnss_n = 1, 1
last_the = est_the
for t in np.arange(0, time, dt):
	if t >= ODO_TIME[odo_n]:
		dx, dy, dthe = ODO_X[odo_n]-ODO_X[odo_n-1], ODO_Y[odo_n]-ODO_Y[odo_n-1], ODO_THE[odo_n] - ODO_THE[odo_n-1]
		dt = ODO_TIME[odo_n]-ODO_TIME[odo_n-1]
		est_delta = math.sqrt(dx**2+dy**2)  + dist_error * dt 
		est_x, est_y = est_x + est_delta * math.cos(est_the), est_y + est_delta * math.sin(est_the)
		est_the += dthe + angle_err * dt
		odo_n += 1
		EST_X, EST_Y, EST_THE = EST_X + [est_x], EST_Y + [est_y], EST_THE + [est_the]
		if gnss_n+1 < len(GNSS_TIME) and t >= GNSS_TIME[gnss_n]:
			if GNSS_REL[gnss_n]:
				err_x, err_y = GNSS_X[gnss_n] - est_x, GNSS_Y[gnss_n] - est_y
				error_the = 0
				if 	gnss_n + 1 < len(GNSS_TIME) and GNSS_REL[gnss_n+1]:
					dx, dy = GNSS_X[gnss_n+1] - GNSS_X[gnss_n], GNSS_Y[gnss_n+1] - GNSS_Y[gnss_n]
					the = math.atan2(dy, dx)
					error_the = the - est_the if math.sqrt(dx**2 + dy**2) > 0.2 else 0
				error_the = (error_the + math.pi) % (2.0 * math.pi) - math.pi
				est_x, est_y, est_the = est_x + gnss_ratio * err_x, est_y + gnss_ratio * err_y, est_the + gnss_ratio * error_the
			gnss_n += 1

last_time = -10
odo_n = 0
EST_ARR_TIME, EST_ARR_X, EST_ARR_Y, EST_ARR_U, EST_ARR_V = [],[],[],[],[]
for t in np.arange(0, time, dt):
	if t >= ODO_TIME[odo_n]:
		odo_n += 1
	if t - last_time >= 1.0:
		last_time = t
		EST_ARR_TIME += [ODO_TIME[odo_n]]
		EST_ARR_X, EST_ARR_Y = EST_ARR_X+[EST_X[odo_n]], EST_ARR_Y+[EST_Y[odo_n]]
		EST_ARR_U, EST_ARR_V = EST_ARR_U + [arrow * math.cos(EST_THE[odo_n])], EST_ARR_V + [arrow * math.sin(EST_THE[odo_n])]

plt.plot(X, Y)
for i in range(len(GNSS_TIME)):
	c = GNSS_I_SCORE[i]/gnss_max_i_score
	plt.scatter(GNSS_X[i], GNSS_Y[i], color=(1-c,1-c,1,1))
for i in range(gnss_reliable_n_min, gnss_reliable_n_max):
	c = GNSS_I_SCORE[i]/gnss_max_i_score
	plt.scatter(GNSS_X[i], GNSS_Y[i], color=(1,1-c,1-c,1))
plt.quiver(ODO_DISP_X, ODO_DISP_Y, ODO_DISP_U, ODO_DISP_V, angles='xy',scale_units='xy',scale=2)
plt.gca().set_aspect('equal', adjustable='box')
#plt.plot(ODO_MOD_X, ODO_MOD_Y)
#plt.plot(ODO_INI_X, ODO_INI_Y)
plt.quiver(EST_ARR_X, EST_ARR_Y, EST_ARR_U, EST_ARR_V, angles='xy',scale_units='xy',scale=2,color=(1,0,0,1))
#plt.plot(EST_X, EST_Y, marker='.')
ax = plt.axes()
for i in range(len(GNSS_TIME)):
	ax.add_patch(patches.Circle((GNSS_X[i], GNSS_Y[i]), GNSS_COV[i], fc="None", alpha = 1, ec="black"))
plt.show()

