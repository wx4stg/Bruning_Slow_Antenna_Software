#!/usr/bin/env python3

import sys
import errno
import json
from time import sleep
import datetime
import RPi.GPIO as GPIO
import subprocess

current_time = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
print(f'[{current_time}] Checking GPS fix...')
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.OUT)
GPIO.output(26, GPIO.LOW)

start_clock_cmd = ['systemctl', 'start', 'update_clock_gps']
check_clock_cmd = ['systemctl', 'status', 'update_clock_gps']
check_test_cmd = ['systemctl', 'status', 'adc_test_startup']
check_data_cmd = ['systemctl', 'status', 'adc_data_collect']
start_data_cmd = ['systemctl', 'start', 'adc_data_collect']

try:
    while True:
        current_time = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
        print(f'[{current_time}] Still waiting for fix...')
        # intended to have gpspipe -w piped to stdin
        line = sys.stdin.readline()
        if not line:
            # parent proc is dead, exit
            break
        # read json gpspipe data
        sentence = json.loads(line)
        current_time = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
        print(f'[{current_time}] Received Sentence: {sentence}')
        # Filter to mode 2 (2d fix) and mode 3 (3d fix)
        if sentence['class'] == 'TPV':
            if sentence['mode'] == 2  or sentence['mode'] == 3:
                if 'time' not in sentence.keys():
                    continue
                sentence_time = datetime.datetime.strptime(sentence['time'][:-5], '%Y-%m-%dT%H:%M:%S')
                current_time = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
                print(f'[{current_time}] GPS fixed, updating clock')
                subprocess.run(start_clock_cmd, stdout=subprocess.DEVNULL)
                while True:
                    print(f'[{current_time}] GPS is ahead of system clock by {(sentence_time - datetime.datetime.utcnow()).total_seconds()} s')
                    if (sentence_time - datetime.datetime.utcnow()).total_seconds() <= 10:
                        current_time = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
                        print(f'[{current_time}] Clock updated successfully')
                        while True:
                            if subprocess.run(check_test_cmd, stdout=subprocess.DEVNULL).returncode != 0:
                                print(f'[{current_time}] ADC test startup not active')
                                if subprocess.run(check_data_cmd, stdout=subprocess.DEVNULL).returncode != 0:
                                    print(f'[{current_time}] Starting data collect!')
                                    subprocess.run(start_data_cmd, stdout=subprocess.DEVNULL)
                                else:
                                    print(f'[{current_time}] Data collect already running.')
                                GPIO.output(26, GPIO.HIGH)
                                break
                            else:
                                current_time = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
                                print(f'[{current_time}] ADC test startup active, waiting')
                                sleep(1)
                        break
                    else:
                        current_time = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')
                        print(f'[{current_time}] Clock not yet updated, waiting...')
                        sleep(1)
                break
except IOError as e:
    # this was stolen from stackoverflow and probably has a good reason to be here
    if e.errno == errno.EPIPE:
        pass
    else:
        raise e
