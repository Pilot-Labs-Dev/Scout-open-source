import time,os
import threading
import signal
import sys
import math
from geometry_msgs.msg import Twist
from rollereye_ros_bridge import *
import datetime
from timeit import default_timer as timer
import json

SC_SOUND_DIR = "/var/roller_eye/"

def disable_print():
  print "print is disalbed"
  sys.stdout = open(os.devnull, 'w')

def enable_print():
  sys.stdout = sys.__stdout__

def direction_2_x_y(degree):
  print "direction_2_x_y"

  rad = degree / 180.0 * math.pi

  if rad <= (math.pi / 2.0):
    return math.sin(rad), math.cos(rad)
  elif rad <= math.pi:
    return math.cos(rad - (math.pi / 2.0)), 0 - math.sin(rad - (math.pi / 2.0))
  elif rad <= (math.pi / 2.0 * 3):
    return  0 - math.sin(rad - math.pi), 0 - math.cos(rad - math.pi)
  else:
    return 0 - math.cos(rad - (math.pi / 2.0 * 3)), math.sin(rad - (math.pi / 2.0 * 3))

def degree_2_rad(degree):
  angle = degree / 180.0 * math.pi
  return angle

class MotionCmdAsyncSender(threading.Thread):
  _ros_bridge = None
  
  _vel = Twist()
  _is_sending = False
  _is_needed_stop = False  #is needed stop translate and rotate
  _is_released = False

  _vel_lock = threading.Lock()
  _is_sending_lock = threading.Lock()
  _is_needed_stop_lock = threading.Lock()
  _is_released_lock = threading.Lock()

  @property 
  def vel(self):
    self._vel_lock.acquire()
    vel = Twist()
    vel.linear.x = self._vel.linear.x
    vel.linear.y = self._vel.linear.y
    vel.linear.z = self._vel.linear.z
    vel.angular.x = self._vel.angular.x
    vel.angular.y = self._vel.angular.y
    vel.angular.z = self._vel.angular.z
    self._vel_lock.release()

    return vel
  
  @vel.setter
  def vel(self, x, y, r):
    self._vel_lock.acquire()
    self._vel.linear.x = x
    self._vel.linear.y = y
    self._vel.linear.z = 0
    self._vel.angular.x = 0
    self._vel.angular.y = 0
    self._vel.angular.z = r
    self._vel_lock.release()

  @vel.setter
  def vel(self, value):
    self._vel_lock.acquire()
    self._vel.linear.x = value.linear.x
    self._vel.linear.y = value.linear.y
    self._vel.linear.z = value.linear.z
    self._vel.angular.x = value.angular.x
    self._vel.angular.y = value.angular.y
    self._vel.angular.z = value.angular.z
    self._vel_lock.release()

  @property
  def is_sending(self):
    self._is_sending_lock.acquire()
    is_sending = self._is_sending
    self._is_sending_lock.release()
    return is_sending

  @is_sending.setter
  def is_sending(self, value):
    self._is_sending_lock.acquire()
    self._is_sending = value
    self._is_sending_lock.release()

  @property
  def is_needed_stop(self):
    self._is_needed_stop_lock.acquire()
    is_needed_stop = self._is_needed_stop
    self._is_needed_stop_lock.release()
    return is_needed_stop

  @is_needed_stop.setter
  def is_needed_stop(self, value):
    self._is_needed_stop_lock.acquire()
    self._is_needed_stop = value
    self._is_needed_stop_lock.release()

  @property
  def is_released(self):
    self._is_released_lock.acquire()
    is_released = self._is_released
    self._is_released_lock.release()
    return is_released

  def release(self):
    self._is_released_lock.acquire()
    self._is_released = True
    self._is_released_lock.release()

  def __init__(self, ros_bridge):
    super(MotionCmdAsyncSender, self).__init__()
    self._ros_bridge = ros_bridge

  def run(self):
    while self.is_released == False:
      if(self.is_needed_stop):
        print("MotionCmdAsyncSender do stop async")

        self._ros_bridge.publish_cmd_vel(0, 0, 0)
        self.is_sending = False
        self.is_needed_stop = False
        print("MotionCmdAsyncSender do stop done")

      if(self.is_sending):
        vel = self._vel
        print("send vel async, x:%.4f y:%.4f z:%.4f" % (vel.linear.x, vel.linear.y, vel.angular.z))
        self._ros_bridge.publish_cmd_vel(vel.linear.x, vel.linear.y, vel.angular.z)

      time.sleep(0.1)
    
    print('MotionCmdAsyncSender thread exit')

  def _create_vel(self, x, y, r):
    vel = Twist()
    vel.linear.x = x
    vel.linear.y = y
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = r
    return vel

  '''
  description: 
  param self
  param speed rotation speed, in rad/s
  param direction 0~360 degree
  '''
  def start_translate(self, speed, direction):
    print "start_translate"

    x_part, y_part = direction_2_x_y(direction)
    
    x = speed * x_part
    y = speed * y_part

    vel = self._create_vel(x, y, 0)
    self.vel = vel
    self.is_sending = True

  '''
  description: 
  param self
  param speed rotation speed, in degee/s
  param direction 0:don't rotate, 1:left, 2:right 
  '''
  def start_rotate(self, speed, direction):
    print "start_rotate"
    rotation_speed = degree_2_rad(speed) 

    if direction == 1:
      rotation_speed = rotation_speed
    elif direction == 2:
      rotation_speed = rotation_speed * -1
    else:
      rotation_speed = 0

    vel = self._create_vel(0, 0, rotation_speed)
    self.vel = vel
    self.is_sending = True

  '''
  description: 
  param self
  param speed in m/s
  param direction 0~360 degree
  param rotation_speed in degree/s
  param rotation_direction  0:don't rotate, 1:left, 2:right 
  '''
  def start_translate_rotate(self, speed, direction, rotation_speed, rotation_direction):
    print "start_translate_rotate"
    x_part, y_part = direction_2_x_y(direction)
    x = speed * x_part
    y = speed * y_part

    rotation_speed = degree_2_rad(rotation_speed)

    if rotation_direction == 1:
      rotation_speed = rotation_speed
    elif rotation_direction == 2:
      rotation_speed = rotation_speed  * -1
    else:
      rotation_speed = 0
      
    vel = self._create_vel(x, y, rotation_speed)
    self.vel = vel
    self.is_sending = True

  '''
  description: thread safe fucntion, stop async translation and rotation, stop sending
  param {*} self
  '''
  def stop_translate_rotate(self):
    self.is_needed_stop = True

    while self.is_needed_stop:
      print "wait async vel cmd sender stop"
      time.sleep(0.01)
    print "async vel cmd sender stop done"


class Rollereye:
  _is_started = False

  _translation_speed = 0      #m/s
  _translation_direction = 0  #0~360 degree

  _rotation_speed = 0         #degree/s
  _rotation_direction = 0     #0:default 1:left 2:right

  _ros_bridge = None
  _lock = threading.Lock()

  _motion_cmd_async_sender = None

  _timer_is_started = False
  _timer_start_time = None
  _timer_pause_time = None

  _ai_detect_sub = None
  _ai_motion_detect_sub = None

  _ai_detect_switch_lock = threading.Lock()
  _ai_detect_result_lock = threading.Lock()

  _ai_detect_switch = dict([("person", False), ("dog", False), ("cat", False), ("motion", False)])
  _ai_detect_result = {"person":{"last_detected_time": -1.0}, "dog":{"last_detected_time": -1.0}, "cat":{"last_detected_time": -1.0}, "motion":{"last_detected_time": -1.0}}

  @property
  def translation_speed(self):
    return self._translation_speed

  @translation_speed.setter
  def translation_speed(self, value):
    self._translation_speed = value

  @property
  def translation_direction(self):
    return self._translation_direction
  
  @translation_direction.setter
  def translation_direction(self, value):
    self._translation_direction = value

  @property
  def rotation_speed(self):
    return self._rotation_speed

  @rotation_speed.setter
  def rotation_speed(self, value):
    self._rotation_speed = value

  @property
  def rotation_direction(self):
    return self._rotation_direction

  @rotation_direction.setter
  def rotation_direction(self, value):
    self._rotation_direction = value

  def _get_ai_detect_switch(self, target):
    self._ai_detect_switch_lock.acquire()
    result = self._ai_detect_switch[target]
    self._ai_detect_switch_lock.release()
    return result

  def _set_ai_detect_switch(self, target, enabled):
    self._ai_detect_switch_lock.acquire()
    self._ai_detect_switch[target] = enabled
    self._ai_detect_switch_lock.release()


  def _get_ai_detect_result_time(self, target):
    self._ai_detect_result_lock.acquire()
    result = self._ai_detect_result[target]["last_detected_time"]
    self._ai_detect_result_lock.release()
    return result

  def _set_ai_detect_result_time(self, target, a_time):
    self._ai_detect_result_lock.acquire()
    self._ai_detect_result[target]["last_detected_time"] = a_time
    self._ai_detect_result_lock.release()

  '''
  description: 
  param {*} self
  param {*} vx
  param {*} vy
  param {*} wz
  param duration unit is ms
  param {*} stop time, unit is ms
  '''  
  def _move_once(self, vx, vy, wz, duration, stop):
    print "do _move_once start, x:%.4fm/s y:%.4fm/s, r:%.4frad/s, duration:%dms, stop:%dms" % (vx, vy, wz, duration, stop)
    PUB_DURATION = 100  #ms
    vel = Twist()

    vel.linear.x = vx
    vel.linear.y = vy
    vel.angular.z = wz

    pubTime = 0  #ms
    
    while pubTime < duration:
      pubTime = pubTime + PUB_DURATION
      self._ros_bridge.publish_raw_cmd_vel(vel)

      if duration - pubTime > 0:
        time.sleep(min(duration - pubTime, PUB_DURATION) / 1000.0)

    if stop > 0:
      vel.linear.x = vel.linear.y = vel.angular.z = 0.0
      self._ros_bridge.publish_raw_cmd_vel(vel)
      time.sleep(stop)
    
    print "do _move_once done"

  '''
  description: 
  param self
  param x m
  param y m 
  param speed m/s
  '''
  def _move(self, x, y, speed):
    distance = math.sqrt(x * x + y * y)
    SPEED_E = 1.0
    if(distance < 1e-3):
      return 0
  
    speed_x = speed * x / distance
    speed_y = speed * y / distance
    duration = max(int(distance / speed * 1000 / SPEED_E), 250)
    self._move_once(speed_x, speed_y, 0.0, duration, 0.05)
    return 0

  '''
  description: 
  param speed_x m/s
  param speed_y m/s
  param speed_w rad/s
  param time in seconds
  '''   
  def _action(self, speed_x, speed_y, speed_w, time):
    self._move_once(speed_x, speed_y, speed_w, time, 0.05)
    return 0

  def _roll(self, angle, rotation_speed, timeout, error):
    self._ros_bridge.call_service_alog_roll(angle, rotation_speed, timeout, error)
    return 0
  
  def float_is_zero(self, value):
    if math.fabs(value) < 0.001:
      return True
    else:
      return False

  def log(self, info):
    print(self)
    print(self.__class__)
    
  '''
  description: Start program
  param self
  return None
  '''  
  def start(self):
    disable_print()
    self._lock.acquire()
    if self._is_started:
      print 'rollereye already started'
    else:
      self._is_started = True
      self._ros_bridge = RollerEyeRosBride()
      self._motion_cmd_async_sender = MotionCmdAsyncSender(self._ros_bridge)
      self._motion_cmd_async_sender.start()
      self._reset_sound_volume_to_default()
      self.record_stop()
      print('rollereye do start')
    self._lock.release()

  '''
  description: release the resource
  param self
  return None
  '''  
  def release(self):
    self.stop_move()
    self._lock.acquire()
    if self._is_started:
      self._motion_cmd_async_sender.release()
      self._motion_cmd_async_sender.join()
      self._ros_bridge.close()
      self._ros_bridge.join()
      self._is_started = False
      print('rollereye do stop')
    else:
     print('rollereye already stop or not started')
    self._lock.release()
    self._reset_sound_volume_to_default()

  '''
  description: Stop the program
  param self
  return None
  '''  
  def stop(self):
    self.release()
    exit()

  '''
  description: Start the timer
  param self
  '''  
  def timerStart(self):
    if self._timer_is_started:
      print('already timerStart')
    else:
      if self._timer_pause_time is None:
        print('do timerStart, no paused time')
        self._timer_start_time = timer()
      else:
        elapsed_time = (self._timer_pause_time - self._timer_start_time)
        self._timer_start_time = timer() - elapsed_time
        self._timer_pause_time = None
        print('do timerStart, pre time is paused')  
      self._timer_is_started = True

  '''
  description: Pause the timer
  param self
  ''' 
  def timerPause(self):
    if self._timer_pause_time is None:
      self._timer_pause_time = timer()
      self._timer_is_started = False
      print('do timerPause')
    else:
      print('already timerPause')
  
  '''
  description: Stop the timer
  param self
  ''' 
  def timerStop(self):
    print('do timerStop')
    self._timer_start_time = None
    self._timer_pause_time = None
    self._timer_is_started = False
  
  '''
  description: Get the total time elapsed from when the timer was started in milliseconds
  param self
  return the time elapsed
  '''
  def getTimerTime(self):
    if self._timer_is_started:
      elapsed_time = (timer() - self._timer_start_time) * 1000
      #print('do getTimerTime:%dms' % elapsed_time)
      return elapsed_time
    else:
      if self._timer_pause_time is not None:
        elapsed_time = (self._timer_pause_time - self._timer_start_time) * 1000
        print('do getTimerTime, time is paused:%dms' % elapsed_time)
        return elapsed_time
      else:
        print('timer not start yet')
        return 0
    
  '''
  description: Get the total time elapsed from when the robot began running up to the current time in milliseconds
  param self
  return the total time elapsed from when the robot began running up to the current time in milliseconds
  '''  
  def getRunTime(self):
    with open('/proc/uptime', 'r') as f:
      uptime_seconds = float(f.readline().split()[0])

    refined_uptime = long(uptime_seconds * 1000L)

    #print('do getRunTime:%f' % refined_uptime)

    return refined_uptime

  '''
  description: Get the system timestamp from 1970-01-01 00:00:00 in milliseconds
  param {*} self
  '''
  def getCurrentTime(self):
    duration = time.time() 
    refined_duration = long(duration * 1000L)
    #print('do getCurrentTime:%ld'% refined_duration)
    return refined_duration
  
  '''
  description: set media value (0:100)
  param self
  param vol volumne to be setted
  '''  
  def set_soundVolume(self, vol):
    refined_vol = int(vol / 100.0 * 100)
    
    print('do set_soundVolume:%d%%, %d' % (vol, refined_vol))

    cmd = "amixer cset numid=3,iface=MIXER,name='Master Playback Volume' " + str(refined_vol)

    print("run: %s" % cmd)

    os.system(cmd)

  def _reset_sound_volume_to_default(self):    
    print("do _reset_sound_volume_to_default")

    cmd = "amixer cset numid=3,iface=MIXER,name='Master Playback Volume' 100"

    print("run: %s" % cmd)

    os.system(cmd)

  
  '''
  description: The robot emits a sound and executes the next command
  param self
  param effect_id int
  param isFinis_finishedished boolean
  '''
  def play_sound(self, effect_id, is_finished):
    if effect_id == 1:
      sound_path = SC_SOUND_DIR + "sc_sound_001.wav"
    elif effect_id == 2:
      sound_path = SC_SOUND_DIR + "sc_sound_002.wav"
    elif effect_id == 3:
      sound_path = SC_SOUND_DIR + "sc_sound_003.wav"
    else:
      print('unknown sound id:%d', effect_id)
    
    if is_finished:
      cmd = "aplay " + sound_path
    else:
      cmd = "aplay " + sound_path + " &"

    print("run: %s" % cmd)
    
    os.system(cmd)
 
  '''
  description: take photo
  param self
  '''  
  def capture(self):
    self._ros_bridge.call_service_record_start(MEDIA_TYPE.PIC, 1, 3, 1)
    time.sleep(3.2) #wait capture finished

  '''
  description: start video recording
  param self
  '''  
  def record_start(self):
    self._ros_bridge.call_service_record_start(MEDIA_TYPE.VIDEO, 1, 300, 0)

  '''
  description: stop video recording
  param self
  ''' 
  def record_stop(self):
    time.sleep(1)
    self._ros_bridge.call_service_record_stop(MEDIA_TYPE.VIDEO)

  '''
  description: set Scout translation speed to (0:1) m/s
  param self
  param speed float
  '''  
  def set_translationSpeed(self, speed):
    print('do set_translationSpeed')
    self.translation_speed = speed
    self._motion_cmd_async_sender.start_translate(self.translation_speed, self.translation_direction)

  '''
  description: set Scout rotation speed to (0:1) degree/s
  param self
  param rotation_speed float
  '''  
  def set_rotationSpeed(self, rotation_speed):
    print('do set_rotationSpeed')
    self.rotation_speed = rotation_speed
    self._motion_cmd_async_sender.start_rotate(self.rotation_speed, self.rotation_direction)


  '''
  description:  set wheel rotation speed(rpm) to front-left/front-right/rear-left/rear-right(-1000:1000)
                The rotaion speed of each wheel can be set independently
  param self
  param frontLeft
  param frontRight
  param rearLeft
  param rearRight
  '''  
  def set_wheel(self, frontLeft, frontRight, rearLeft, rearRight):
    print('to do set_wheel')

  '''
  description: set Scount to translate at (0:360)
  param self
  param degree unit is degree
  '''  
  def set_translate(self, degree):
    print('set_translate')
    #self._stop_async_translate_rotate()
    self.translation_direction = degree
    self._motion_cmd_async_sender.start_translate(self.translation_speed, self.translation_direction)

  def _stop_async_translate_rotate(self):
    print "do stop async translate and rotate"
    self._motion_cmd_async_sender.stop_translate_rotate()

  '''
  description: set Scount to translate at (0:360) for (0:20) s
  param self
  param degree
  param seconds
  '''  
  def set_translate_2(self, degree, seconds):
    print('set_translate_2')
    if(degree < 0 or degree > 360):
      print('invalid degree:%d, ignore set_translate_2' % degree)
      return

    if(seconds < 0 or seconds > 20):
      print('invalid seconds:%d, ignore set_translate_2' % seconds)
      return

    self._stop_async_translate_rotate()
    self.translation_direction = degree

    speed = self.translation_speed

    x_part, y_part = direction_2_x_y(degree)

    x_distance = x_part * speed * seconds
    y_distance = y_part * speed * seconds

    print('set_translate_2, degree:%.2f, seconds:%d, x_distance:%.2f, y_distance:%.2f, speed:%.2f' % (degree, seconds, x_distance, y_distance, speed))

    self._move(x_distance, y_distance, speed)

  '''
  description: set Scount to translate at (0:360) for (0:5) meters
  param self
  param degree
  param speed int
  '''  
  def set_translate_3(self, degree, meters):
    print('set_translate_3')
    if(degree < 0 or degree > 360):
      print('invalid degree:%d, ignore set_translate_3' % degree)
      return

    if(meters < 0 or meters > 5):
      print('invalid meters:%d, ignore set_translate_3' % meters)
      return

    self._stop_async_translate_rotate()
    self.translation_direction = degree

    speed = self.translation_speed

    x_part, y_part = direction_2_x_y(degree)

    x_distance = x_part * meters
    y_distance = y_part * meters

    self._move(x_distance, y_distance, speed)

  '''
  description: set Scount to translate at (0:360) for (0:1) m/s
  param self
  param degree
  param speed float
  '''  
  def set_translate_4(self, degree, speed):
    print('set_translate_4')
    if(degree < 0 or degree > 360):
      print('invalid degree:%d, ignore set_translate_4' % degree)
      return

    if(speed < 0 or speed > 1):
      print('invalid speed:%.2f, ignore set_translate_4' % speed)
      return

    self.translation_direction = degree
    self.translation_speed = speed

    self._motion_cmd_async_sender.start_translate(self.translation_speed, self.translation_direction)

  '''
  description: set Scount to rotate(left/right)
  param self
  param {*} direction
  '''  
  def set_rotate(self, direction):
    print('set_rotate')

    #if(direction < 0 or direction > 2):
    #  print('invalid direction:%d, ignore set_rotate' % direction)
    #  return

    self.rotation_direction = direction    
    self._motion_cmd_async_sender.start_rotate(self.rotation_speed, self.rotation_direction)

  '''
  description: set Scount to rotate(left/right) for (0:20)s
  param self
  param direction int
  param seconds int
  '''  
  def set_rotate_2(self, direction, seconds):
    print('set_rotate_2')

    if(direction < 0 or direction > 2):
      print('invalid direction:%d, ignore set_rotate_2' % direction)
      return

    if(seconds < 0 or seconds > 20):
      print('invalid seconds:%.2f, ignore set_rotate_2' % seconds)
      return

    self._stop_async_translate_rotate()
    self.rotation_direction = direction
    rotation_speed = self.rotation_speed

    if(self.rotation_direction == 1):
      angle = rotation_speed * seconds
    else:
      angle = rotation_speed * seconds * -1
      

    if(self.float_is_zero(angle)):
      print("angle is zero")
      return

    print('set_rotate_2, rotation_speed:%.2fdegree/s, angle:%.2f degee' % (rotation_speed, angle))

    timeout = seconds * 1000

    self._ros_bridge.call_service_alog_roll(degree_2_rad(angle), degree_2_rad(rotation_speed), timeout)

  '''
  description: set Scount to rotate(left/right) for (0:360)
  param self
  param direction int
  param degree int 
  '''  
  def set_rotate_3(self, direction, degree):
    print('set_rotate_3')

    if(direction < 0 or direction > 2):
      print('invalid direction:%d, ignore set_rotate_3' % direction)
      return

    if(degree < 0 or degree > 360):
      print('invalid degree:%d, ignore set_rotate_3' % degree)
      return

    self._stop_async_translate_rotate()
    self.rotation_direction = direction
    rotation_speed = self.rotation_speed

    targetDegree = degree

    if(self.rotation_direction == 1):
      targetDegree = degree
    else:
      targetDegree = degree * -1
    
    if(self.float_is_zero(rotation_speed)):
      print("rotation_speed is zero")
      return

    timeout = degree / (float)(self.rotation_speed) * 1000

    if(timeout < 10000):
      print("set_rotate_3, timeout %dms less than 10s, set to 10s" % timeout)
      timeout = 10000

    print("set_rotate_3, rotation_speed:%.2f, angle:%.2f, timeout:%dms" % (rotation_speed, targetDegree, timeout))

    self._ros_bridge.call_service_alog_roll(degree_2_rad(targetDegree), degree_2_rad(rotation_speed), timeout)

  '''
  description: set Scount to translate towards front at (0:360) and rotate (left/right) 
               let the Scout to move in a specific direction while ratating simutaneously
  param self
  param direction rotation direction
  param degree translate direction
  '''  
  def set_translate_rotate(self, direction, degree):
    print('set_translate_rotate')

    if(degree < 0 or degree > 360):
      print('invalid degree:%d, ignore set_translate_rotate' % degree)
      return

    if(direction < 0 or direction > 2):
      print('invalid direction:%d, ignore set_translate_rotate' % direction)
      return

    #self._stop_async_translate_rotate()

    self.translation_direction = degree
    self.rotation_direction = direction

    self._motion_cmd_async_sender.start_translate_rotate(self.translation_speed, self.translation_direction, self.rotation_speed, self.rotation_direction)

  '''
  description: stop move
  param self
  '''  
  def stop_move(self):
    print('stop_move')
    self._stop_async_translate_rotate()

  '''
  description: enable the Scout visual identification funtion for vision person, cat ,dog
  param self
  param target, reg.person/reg.cat/reg.dog
  '''  
  def enable_reg(self, target):
    self._set_ai_detect_setting(target, True)

    if target == reg.motion:
      self._enable_ai_motion_detect()
    else:
      self._enable_ai_detect()

  '''
  description: disable the Scout visual identification funtion for vision person, cat ,dog
  param self
  param target, reg.person/reg.cat/reg.dog
  '''  
  def disable_reg(self, target):
    self._set_ai_detect_setting(target, False)

    if (self._get_ai_detect_switch('person') == False) \
       and (self._get_ai_detect_switch('dog') == False) \
       and (self._get_ai_detect_switch('cat') == False):
      print('disable_detection all')
      self._disable_ai_detect()

    if(self._get_ai_detect_switch('motion') == False):
      print('disable_detection motion')
      self._disable_ai_motion_detect()
      

  '''
  description: when specific information is identified, the corresponding block runs its internal program
  param self
  return the target(reg.person/reg.dog/reg.cat) recognised
  '''  
  def recogResult(self):
    return self.get_ai_last_detect_result()
    

  '''
  description: when the specific information(person/cat/dog,etc.)is indentified, the condition is return True. Otherwise, it is returned False.
  param self
  param {*} target, reg.person/reg.cat/reg.dog/...
  return true or false
  '''  
  def recResult(self, target):
    if target == self.recogResult():
      return True
    else:
      return False
  
  '''
  description: when specific information is identified, the Scout continues executing commands. Otherwise, it continues waiting
  param self
  param {*} target, reg.person/reg.cat/reg.dog/...
  return None
  '''  
  def recWait(self, target):
    print("recWait:%d" % target)
    self.enable_reg(target)

    while True:
      if target == self.recogResult():
        print("recWait done:%d" % target)
        return True
      else:
        time.sleep(0.05)

  '''
  description: enable motion detect
  param self
  return {*}
  '''  
  def enable_detection(self):
    print('enable_detection')
    self.enable_reg(reg.motion)

  '''
  description: disable motion detect
  param self
  return {*}
  '''  
  def disable_detection(self):
    print('disable_detection')
    self.disable_reg(reg.motion)

  '''
  description: when detected motion,the corresponding block runs its internal program
  param self
  return true when detected, false when not detected
  '''  
  def motionDetected(self):
    print('motionDetected')
    if reg.motion == self.get_ai_last_motion_detect_result():
      return True
    else:
      return False

  def _enable_ai_detect(self):
    if self._ai_detect_sub is None:
      self._ai_detect_sub = self._ros_bridge.sub_topic_ai_detect(self._ai_detect_cb)
      print("_enable_ai_detect")
  
  def _enable_ai_motion_detect(self):
    if self._ai_motion_detect_sub is None:
      self._ai_motion_detect_sub = self._ros_bridge.sub_topic_ai_motion_detect(self._ai_motion_detect_cb)
      print("_enable_ai_motion_detect")
    
  def _disable_ai_detect(self):
    if self._ai_detect_sub is not None:
      self._ai_detect_sub.unregister()
      self._ai_detect_sub = None
  
  def _disable_ai_motion_detect(self):
    if self._ai_motion_detect_sub is not None:
      self._ai_motion_detect_sub.unregister()
      self._ai_motion_detect_sub = None


  def _ai_detect_cb(self, detected_data):
    print('_ai_detect_cb:%s' % detected_data.name)

    if (detected_data.name == "person") and self._get_ai_detect_switch("person"):
      print('deteceted:%s' % detected_data.name)
      self._set_ai_detect_result_time("person", timer())
    
    if (detected_data.name == "dog") and self._get_ai_detect_switch("dog"):
      print('deteceted:%s' % detected_data.name)
      self._set_ai_detect_result_time("dog", timer())

    if (detected_data.name == "cat") and self._get_ai_detect_switch("cat"):
      print('deteceted:%s' % detected_data.name)
      self._set_ai_detect_result_time("cat", timer())
  
  def _ai_motion_detect_cb(self, detected_data):
    print('_ai_motion_detect_cb:%s' % detected_data.name)
    if (detected_data.name == "motion") and self._get_ai_detect_switch("motion"):
      print('motion detected')
      self._set_ai_detect_result_time("motion", timer())

  def get_ai_last_detect_result(self):
    person_detect_time = long(self._get_ai_detect_result_time("person") * 1000)
    dog_detect_time    = long(self._get_ai_detect_result_time("dog") * 1000)
    cat_detect_time    = long(self._get_ai_detect_result_time("cat") * 1000)

    last_detected_time = max((person_detect_time, dog_detect_time, cat_detect_time))

    elapsed_time = long(timer() * 1000)- last_detected_time 

    if(elapsed_time < 1000): #ms
      if last_detected_time == person_detect_time:
        #print("last detect result is person in %ldms ago" % elapsed_time)
        #self._set_ai_detect_result_time("person", -1)
        return reg.person
      elif last_detected_time == dog_detect_time:
        #print("last detect result is dog in %ldms ago" % elapsed_time)
        #self._set_ai_detect_result_time("dog", -1)
        return reg.dog
      elif last_detected_time == cat_detect_time:
        #print("last detect result is cat in %ldms ago" % elapsed_time)
        #self._set_ai_detect_result_time("cat", -1)
        return reg.cat
      else:
        return None
    else:
        return None
    
  def get_ai_last_motion_detect_result(self):
    motion_detect_time = long(self._get_ai_detect_result_time("motion")* 1000)

    last_detected_time = motion_detect_time

    elapsed_time = long(timer() * 1000)- last_detected_time 

    if(elapsed_time < 1000): #ms
        return reg.motion
    else:
        return None

  '''
  description: 
  param  setting ai detect setting in dict
  return True: setting OK, False: setting failed
  '''  
  def _set_ai_detect_setting(self, target, enabled):
    if (target != reg.person) and (target != reg.dog) and (target != reg.cat) and (target != reg.motion):
      print("invalid detect target")
      return False
    
    if (target == reg.motion):
      self._set_ai_detect_switch('motion', enabled)
    elif(target == reg.person):
      self._set_ai_detect_switch('person', enabled)
    elif(target == reg.dog):
      self._set_ai_detect_switch('dog', enabled)
    else:
      self._set_ai_detect_switch('cat', enabled)
    
    return True

  def handle_exception(self, e):
    self._ros_bridge.call_service_programming_exception_handle(e)
    
  def handle_meta(self, msg):
    if(self._is_started):
      self._ros_bridge.call_service_programming_meta_handle(msg)

  def handle_msg(self, msg_type, msg):
    refined_msg = str(msg)
    if(self._is_started):
      self._ros_bridge.call_service_programming_msg_handle(msg_type, refined_msg)

def signalHandler(signalNum, frame):
  print("signalHandler bye")
  rollereye.stop()

rollereye = Rollereye()

def highlightBlock(meta_msg):
  time.sleep(0.05)
  rollereye.handle_meta(meta_msg)

def rollereye_print(msg):
  time.sleep(0.05)
  rollereye.handle_msg(0, msg)

# def start():
#   rollereye.set_rotationSpeed(60)
#   rollereye.set_translationSpeed(0.3)
#   rollereye.set_translate_rotate(1,90)
#   rollereye.timerStart()
#   while rollereye.getTimerTime() <= 10000:
#     pass

# if __name__ == '__main__':
#    rollereye.start()
#    start()
#    rollereye.stop()

if __name__ == '__main__':
  signal.signal(signal.SIGINT, signalHandler) 
  signal.signal(signal.SIGHUP, signalHandler)
  signal.signal(signal.SIGTERM, signalHandler)

  rollereye.start()

  #rollereye._enable_ai_detect()

  rollereye.timerStart()



  while rollereye.getTimerTime() < 100000:
    if(rollereye.getTimerTime() > 5000):
      rollereye.stop()
  


  rollereye.enable_detection()

  rollereye.enable_reg(target = reg.person)
  rollereye.enable_reg(target = reg.dog)
  rollereye.disable_reg(target = reg.cat)

  rollereye.disable_reg(target = reg.person)
  rollereye.disable_reg(target = reg.dog)
  rollereye.disable_reg(target = reg.cat)

  rollereye.enable_reg(target = reg.cat)
  rollereye.enable_reg(target = reg.person)
  #rollereye.recWait(target = reg.person)

  rollereye.disable_reg(target = reg.person)
  rollereye.disable_reg(target = reg.dog)
  rollereye.disable_reg(target = reg.cat)

  rollereye.disable_detection()

  rollereye.enable_reg(target = reg.person)
  #rollereye.recWait(target = reg.person)
  
  rollereye.enable_reg(target = reg.person)

  test = timer()
  time.sleep(1)
  test2 = timer()

  vol = 100

  rollereye.play_sound(1 + vol % 3, False)

  while vol > 90:
    rollereye.set_soundVolume(vol)
    rollereye.play_sound(1 + vol % 3, True)
    vol = vol - 10

    rollereye._reset_sound_volume_to_default()


  cnt = 0
  while 1:
    time.sleep(0.1)
    rollereye.recogResult() 
    continue

    cnt += 1
    #print('sleep:%d' % cnt)
    time.sleep(1)
    rollereye.getRunTime()
    rollereye.getCurrentTime()
    rollereye.set_soundVolume(0)
    rollereye.play_sound(0, True)
    rollereye.capture() 
    rollereye.capture() 
    rollereye.capture() 
    rollereye.record_start()
    time.sleep(5)
    rollereye.record_stop()
    rollereye.capture() 
    rollereye.capture() 
    rollereye.record_start()
    time.sleep(10)
    rollereye.record_stop()
    rollereye.capture() 

    rollereye.timerStart()
    time.sleep(2.89712)
    rollereye.getTimerTime()
    rollereye.timerPause()
    time.sleep(6.54323)
    rollereye.getTimerTime()
    rollereye.timerStart()
    time.sleep(5.1)
    rollereye.getTimerTime()
    rollereye.timerStop()
    rollereye.getTimerTime()

    rollereye.set_translationSpeed(0.2)
    rollereye.set_rotationSpeed(5)
    rollereye.set_wheel(frontLeft = 0, frontRight = 0, rearLeft = 0, rearRight = 0)
    rollereye.set_translate(degree = 1)

    time.sleep(1)

    rollereye.set_translate_2(degree = 270, seconds = 1) 
    rollereye.set_translate_3(degree = 45, meters = 1)
    rollereye.set_translate_4(degree = 0, speed = 0.1)
    rollereye.set_rotate(direction = 1)
    rollereye.set_rotate_2(direction = 1, seconds = 10)
    rollereye.set_rotate_3(direction = 1, degree = 30)
    rollereye.set_translate_rotate(degree = 30, direction = 1)
    rollereye.stop_move()

    rollereye.recogResult() 
    rollereye.recResult(target = reg.person)
    rollereye.recWait(target = reg.person)
    rollereye.enable_detection()
    rollereye.disable_detection() 
    rollereye.motionDetected()

    if(cnt > 60):
      break
  rollereye.stop()
  print "main thread exit"