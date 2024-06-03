import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import csv, sys
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.lines as mlines

from pandas import *


def st1_bounds(distance_cw,speed,capacity_w,nu_c,nu_t,nu_w,nu_min, N_w,sigma_w,tau):

	delta_t = distance_cw/speed

	delta_c = capacity_w / (nu_c-nu_min)

	delta_w = (capacity_w - 2 * (nu_t+nu_min) * delta_t)/(nu_w + nu_min)

	D = delta_w/(delta_w + 2 * delta_t + delta_c)

	WP = D * N_w * tau * sigma_w

	E = 1.0 - ( (2 * (nu_t+nu_min) * delta_t) + nu_min * (delta_w+delta_c))/capacity_w

	return [WP,E]

# ---------------------------------------------------------------------------
distance_cw = 0 # (cm) distance between charging and work locations

distance_sw = 30 #(cm) distance between supply and work locations

distance_cs = 100 # (cm)

speed= 12 #(cm/s) robot speed

capacity_w= 100.0 # worker capacity

nu_c = 10 # charging rate

nu_t = 1.0 # rate of consuming energy when travelling

nu_w = 1.0 * nu_t # rate of consuming energy when whork

nu_min = 0.05 # how much a robot consumes when idle

N_w =  6 # number of workers

sigma_w = 10 # rate of performing work

tau = 600 # (s) experiment length

# ---------------------------------------------------------------------------

st1_work_performed , st1_energy_efficiency = st1_bounds(distance_cw,speed,capacity_w,nu_c,nu_t,nu_w,nu_min, N_w,sigma_w,tau)

print(st1_work_performed,st1_energy_efficiency)