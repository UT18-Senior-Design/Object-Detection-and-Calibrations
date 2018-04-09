#!/usr/bin/python

"""
    read or unpack Velodyne VLP-16 data
    usage:
        ./velodyne.py <read | unpack> [bin file dir]
"""

import os
import csv
import sys
import socket
import glob
from datetime import datetime, timedelta
import struct
import time
import traceback
import numpy as np
from multiprocessing import Process, Queue, Pool

import logging
import logging.config

HOST = "192.168.1.201"
PORT = 2368

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16

EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000

DATA_QUEUE = Queue(-1)

formatter = '[%(asctime)s][%(filename)s:%(lineno)s][%(levelname)s][%(message)s]'

LOGGING_CONFIG = {
    'version': 1,
    'disable_existing_loggers': False,  # this fixes the problem

    'formatters': {
        'standard': {
            'format': formatter,
        },
    },
    'handlers': {
        'default': {
            'level': 'DEBUG',
            'class': 'logging.StreamHandler',
            'formatter': 'standard'
        },
        "debug_file_handler": {
            "class": "logging.handlers.TimedRotatingFileHandler",
            "level": "DEBUG",
            "formatter": "standard",
            "filename": "./logs/lidar.log",
            "when": "D",
            "interval": 1,
            "backupCount": 30,
            "encoding": "utf8"
        },
    },
    'loggers': {
        '': {
            'handlers': ["default", 'debug_file_handler'],
            'level': 'DEBUG',
            'propagate': False
        },
    }
}

logging.config.dictConfig(LOGGING_CONFIG)
logger = logging.getLogger("")

def save_csv(path, data):
    with open(path, 'w') as fp:
        wr = csv.writer(fp, delimiter=',')
        wr.writerows(data)

def calc(dis, azimuth, laser_id, timestamp):
    R = dis * DISTANCE_RESOLUTION
    omega = LASER_ANGLES[laser_id] * np.pi / 180.0
    alpha = azimuth / 100.0 * np.pi / 180.0
    X = R * np.cos(omega) * np.sin(alpha)
    Y = R * np.cos(omega) * np.cos(alpha)
    Z = R * np.sin(omega)
    return [X, Y, Z, timestamp]

def unpack(dirs):
    files = glob.glob(dirs + '/*.bin')
    points = []
    scan_index = 0
    prev_azimuth = None
    for x in files:
        d = open(x, 'rb').read()
        n = len(d)
        for offset in xrange(0, n, 1223):
            ts = d[offset : offset + 17]
            data = d[offset + 17 : offset + 1223]
            print (ts, len(data))
            timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
            assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
            timestamp = float(ts)
            seq_index = 0
            for offset in xrange(0, 1200, 100):
                flag, azimuth = struct.unpack_from("<HH", data, offset)
                assert flag == 0xEEFF, hex(flag)
                for step in xrange(2):
                    seq_index += 1
                    azimuth += step
                    azimuth %= ROTATION_MAX_UNITS
                    if prev_azimuth is not None and azimuth < prev_azimuth:
                        file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
                        path = datetime.now().strftime(file_fmt)
                        try:
                            if os.path.exists(path) is False:
                                os.makedirs(path)
                        except Exception:
                            pass;
                        #     print (e)
                        if not points:
                            timestamp_str = '%.6f' % time.time()
                        else:
                            timestamp_str = '%.6f' % points[0][3]
                        csv_index = '%08d' % scan_index
                        save_csv("{}/i{}_{}.csv".format(path, csv_index, timestamp_str), points)
                        logger.info("{}/i{}_{}.csv".format(path, csv_index, timestamp_str))
                        scan_index += 1
                        points = []
                    prev_azimuth = azimuth
                    # H-distance (2mm step), B-reflectivity (0
                    arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
                    for i in xrange(NUM_LASERS):
                        time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0
                        if arr[i * 2] != 0:
                            points.append(calc(arr[i * 2], azimuth, i, timestamp + time_offset))

def save_package(dirs, data_queue):
    try:
        if os.path.exists(dirs) is False:
            os.makedirs(dirs)
        cnt = 0
        fp = None
        while True:
            if data_queue.empty():
                pass
            else:
                msg = data_queue.get()
                data = msg['data']
                ts = msg['time']
                print (ts, len(data), 'queue size: ', data_queue.qsize(), cnt)
                if fp == None or cnt == 1000000:
                    if fp != None:
                        fp.close()
                    file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
                    path = str(datetime.now().strftime(file_fmt)) + '.bin'
                    logger.info('save to' + path)
                    print ('save to ', path)
                    fp = open(path, 'ab')
                    cnt = 0
                cnt += 1
                fp.write('%.6f' % ts)
                fp.write(data)
    except KeyboardInterrupt:
        pass
        #print (e)
    finally:
        if fp != None:
            fp.close()

def capture(port, data_queue):
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(('', port))
    try:
        while True:
            try:
                data = soc.recv(2000)
                if len(data) > 0:
                    assert len(data) == 1206, len(data)
                    data_queue.put({'data': data, 'time': time.time()})
            except Exception:
                pass
            #     print (dir(e), e.message, e.__class__.__name__)
            #     traceback.print_exc(e)
    except KeyboardInterrupt:
        pass
        #print (e)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print (__doc__)
        sys.exit(2)
    if sys.argv[1] == 'read':
        top_dir = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        processA = Process(target = capture, args = (PORT, DATA_QUEUE))
        processA.start()
        processB = Process(target = save_package, args = (sys.argv[2] + '/' + top_dir, DATA_QUEUE))
        processB.start()
    else:
        unpack(sys.argv[2])