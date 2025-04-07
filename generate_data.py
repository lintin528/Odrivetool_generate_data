import odrive
from odrive.enums import *
import time
import csv
import random
import numpy as np
import os
import datetime


# Find a connected ODrive (this will block until you connect one)
print("Finding an ODrive...")
odrv0 = odrive.find_any()
folder_path = './data'
if not os.path.exists(folder_path):
    os.makedirs(folder_path)

if odrv0:
    print(f"find motor ")

def sine_wave_input(time, frequency=5, amplitude=1):
    """ generate (Sine Wave) input signal """
    return amplitude * np.sin(2 * np.pi * frequency * time)

def three_stage_ramp(time, t_peak1, t_peak2, slope_up1=1, slope_down=-1, slope_up2=0.5):
    """ generate (Ramp) input signal """
    y = np.piecewise(time,
        [time < t_peak1, 
         (time >= t_peak1) & (time < t_peak2), 
         time >= t_peak2],
        [lambda t: slope_up1 * t,  
         lambda t: slope_down * (t - t_peak1) + slope_up1 * t_peak1,  
         lambda t: slope_up2 * (t - t_peak2) + (slope_down * (t_peak2 - t_peak1) + slope_up1 * t_peak1)]
    )
    
    return y

get_input = {
    'sine': sine_wave_input,
    'three_stage_ramp': three_stage_ramp,
}

def run_motor_control(input_name, i, folder_path, get_input_fn, **input_params):
    timestamp_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    file_name = f"{folder_path}/{input_name}_input_{i}_{timestamp_str}.csv"
    print(f"Run {input_name} input round {i}\n")

    with open(file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Timestamp', 'Encoder Position', 'Target Position'])

        start_time = time.time()
        start_pos = odrv0.axis0.pos_estimate
        position = start_pos

        # Setup ODrive for position control
        odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        odrv0.axis0.controller.input_pos = position

        target_positions = get_input_fn(time_array, **input_params)
        for j, offset in enumerate(target_positions):
            timestamp = time.time() - start_time

            position = start_pos + offset
            odrv0.axis0.controller.input_pos = position
            encoder_pos = odrv0.axis0.pos_estimate

            writer.writerow([timestamp, encoder_pos - start_pos, offset])
            time.sleep(0.1)  # Adjust as needed


# Open a CSV file to write the data
iteration_times = 500
Ts = 0.1
time_array = np.arange(0, 12, Ts)

for i in range(iteration_times):
    freq = np.random.uniform(0.5, 1)
    sine_amplitude = np.random.uniform(0.5, 1)

    t_peak1 = np.random.uniform(2, 5)
    t_peak2 = np.random.uniform(t_peak1 + 4, 6)
    slope_up1 = np.random.uniform(1, 2)
    slope_down = np.random.uniform(-2, -0.5)
    slope_up2 = np.random.uniform(0.5, 3)

    run_motor_control(
        input_name='sine',
        i=i,
        folder_path=folder_path,
        get_input_fn=get_input['sine'],
        frequency=freq,
        amplitude=sine_amplitude
    )

    run_motor_control(
        input_name='three_stage_ramp',
        i=i,
        folder_path=folder_path,
        get_input_fn=get_input['three_stage_ramp'],
        t_peak1=t_peak1,
        t_peak2=t_peak2,
        slope_up1=slope_up1,
        slope_down=slope_down,
        slope_up2=slope_up2
    )
        

print("Data collection complete.")
