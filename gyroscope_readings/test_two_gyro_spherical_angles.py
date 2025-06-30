import time
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def process_gyroscope(t_count, angleX_50, angleY_50, angleZ_50, 
                      angleX_51, angleY_51, angleZ_51, record):

    temp_record = [t_count]
    print(t_count, angleX_50, angleY_50, angleZ_50, angleX_51, angleY_51, angleZ_51)

    temp_record.extend([angleX_50, angleY_50, angleZ_50])
    tilting_angle50 = np.arccos( np.cos(angleX_50/180*np.pi) * np.cos(angleY_50/180*np.pi)) # in rad
    temp_record.extend([angleX_51, angleY_51, angleZ_51])
    tilting_angle51 = np.arccos( np.cos(angleX_51/180*np.pi) * np.cos(angleY_51/180*np.pi))
    temp_record.extend([tilting_angle50*180/np.pi, tilting_angle51*180/np.pi])

    tilting_diff = -tilting_angle50 + tilting_angle51 # *-1: depends on X rotation direction 
    Z_diff = angleZ_50 - angleZ_51 # in deg
    temp_record.extend([tilting_diff*180/np.pi, Z_diff])

    record.append(temp_record)

    return record

def printcsv(filename, results):
    header = ['Time (s)']
    for gyro_num in range(2):
        header.extend([
            f'Roll/X rotation of gyro {gyro_num+50} (°)',
            f'Pitch/Y rotation of gyro {gyro_num+50} (°)',
            f'Yaw/Z rotation of gyro {gyro_num+50} (°)'
            ])
        gyro_num += 1
    for gyro_num in range(2):
        header.extend([
            f'Resultant tilting angle of gyro {gyro_num+50} from ground (°)'
            ])
        gyro_num += 1
    header.extend(["Difference in tilting (°)"])
    header.extend(["Difference in Z rotations (°)"])
    header.extend([
        f'X acceleration of gyro 50 (*9.81 m/s^2)',
        f'Y acceleration of gyro 50 (*9.81 m/s^2)',
        f'Z acceleration of gyro 50 (*9.81 m/s^2)'
        ])
    
    timestamp = time.strftime("%m%d_%H%M%S")
    outputfile = "{}_{}".format(timestamp, filename)
    folder_path = os.path.join('gyro_records')
    os.makedirs(folder_path, exist_ok=True)
    output_file = os.path.join(folder_path, outputfile)

    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(results)
    return header

def read_gyroscope(start_count):
    angleX_50, angleY_50, angleX_51, angleY_51 = 0, 0, 0, 0 
    angleZ_50, angleZ_51 = 0, 0
    
    angleX_50 += 1*start_count; angleX_51 += 3*start_count
    angleY_50 += 1*start_count; angleY_51 += 3*start_count
    angleZ_50 += 3*start_count; angleZ_51 += 4*start_count
    start_count += 1
    return angleX_50, angleY_50, angleZ_50, angleX_51, angleY_51, angleZ_51, start_count

def plot_graphs(results, header):
    results = np.asarray(results)       
    timestamp = results[:, 0]    

    fig = plt.figure(figsize=(12, 8))
    outer = gridspec.GridSpec(
        nrows=1, ncols=2, width_ratios=[1, 2], wspace=0.30   # more space between cols
    )

    left = outer[0].subgridspec(3, 1, hspace=0.45) 
    left_pairs = [(1, 4), (2, 5), (3, 6)] 
    for row, (c1, c2) in enumerate(left_pairs):
        ax = fig.add_subplot(left[row, 0])
        ax.plot(timestamp, results[:, c1], label=header[c1])
        ax.plot(timestamp, results[:, c2], label=header[c2])
        ax.set_ylabel("Value")
        if row == 2:                       # bottom left plot gets the time label
            ax.set_xlabel("Time")
        ax.legend(fontsize="small")

    right = outer[1].subgridspec(2, 1, hspace=0.45)          # 2 rows × 1 col
    right_pairs = [(7, 8), (9, 10)]       # columns 8&9, 10&11 in 0-based form
    for row, (c1, c2) in enumerate(right_pairs):
        ax = fig.add_subplot(right[row, 0])
        ax.plot(timestamp, results[:, c1], label=header[c1])
        ax.plot(timestamp, results[:, c2], label=header[c2])
        ax.set_ylabel("Value")
        if row == 1:                       # bottom right plot gets the time label
            ax.set_xlabel("Time")
        ax.legend(fontsize="small")

    fig.suptitle("Gyroscope data vs Time", fontsize=14, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])  # keep room for the suptitle
    plt.show()
    return

def main1():

    # Define
    filename="two_gyro_angles_test.csv"
    t_count = 0; results = []
    start_count = 0

    # Run
    while t_count < 100:
        
        angleX_50, angleY_50, angleZ_50, \
            angleX_51, angleY_51, angleZ_51, start_count = read_gyroscope(start_count)
        results = process_gyroscope(t_count, 
                                    angleX_50, angleY_50, angleZ_50, angleX_51, angleY_51, angleZ_51,
                                    results)
        t_count += 1

    header = printcsv(filename, results)
    plot_graphs(results, header)

main1()